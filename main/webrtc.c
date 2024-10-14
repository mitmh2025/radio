#include "webrtc.h"
#include "main.h"

#include "esp_check.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_log.h"

#include "opus.h"

#include "com/amazonaws/kinesis/video/common/Include.h"
#include "com/amazonaws/kinesis/video/webrtcclient/Include.h"

struct webrtc_connection {
  webrtc_connection_state_change_callback_t state_change_callback;
  webrtc_connection_buffer_duration_callback_t buffer_duration_callback;
  void *user_data;

  PRtcPeerConnection peer_connection;
  PRtcRtpTransceiver transceiver;
  char *session_url;
  esp_http_client_handle_t whep_client;
  webrtc_connection_state_t state;

  char *local_ice_ufrag;
  char *local_ice_pwd;
  char *local_media_line;

  // Lock to protect access to pending_candidates
  SemaphoreHandle_t lock;
  // We should never have this many pending candidates, but it's a relatively
  // cheap allocation
  char *pending_candidates[16];

  OpusDecoder *decoder;
  char *last_received_packet;
  size_t last_received_packet_len;
  size_t last_received_packet_capacity;
};

struct webrtc_connect_http_data {
  webrtc_connection_t connection;
  PRtcSessionDescriptionInit remoteDescription;
};

void webrtc_free_connection(webrtc_connection_t connection) {
  if (!connection) {
    return;
  }

  free(connection->last_received_packet);

  if (connection->decoder) {
    opus_decoder_destroy(connection->decoder);
  }

  esp_http_client_cleanup(connection->whep_client);
  free(connection->session_url);
  free(connection->local_ice_ufrag);
  free(connection->local_ice_pwd);
  free(connection->local_media_line);

  for (int i = sizeof(connection->pending_candidates) /
                   sizeof(connection->pending_candidates[0]) -
               1;
       i >= 0; i--) {
    free(connection->pending_candidates[i]);
  }

  freeTransceiver(&connection->transceiver);

  freePeerConnection(&connection->peer_connection);

  if (connection->lock) {
    xSemaphoreTake(connection->lock, portMAX_DELAY);
    vSemaphoreDelete(connection->lock);
  }

  free(connection);
}

static VOID webrtc_logger(UINT32 level, const PCHAR tag, const PCHAR fmt, ...) {
  int esp_log_level = 0;
  // kvswebrtc is pretty noisy so we'll cram down some of the log levels
  switch (level) {
  case LOG_LEVEL_VERBOSE:
  case LOG_LEVEL_DEBUG:
    esp_log_level = ESP_LOG_VERBOSE;
    break;
  case LOG_LEVEL_INFO:
  case LOG_LEVEL_WARN:
  case LOG_LEVEL_ERROR:
    esp_log_level = ESP_LOG_DEBUG;
    break;
  case LOG_LEVEL_FATAL:
    esp_log_level = ESP_LOG_ERROR;
    break;
  default:
    esp_log_level = ESP_LOG_VERBOSE;
    break;
  }

  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(NULL, 0, fmt, args);
  va_end(args);
  if (len < 0) {
    return;
  }

  char *buffer = malloc(len + 1);
  if (!buffer) {
    return;
  }

  va_start(args, fmt);
  len = vsnprintf(buffer, len + 1, fmt, args);
  va_end(args);

  if (len < 0) {
    free(buffer);
    return;
  }

  ESP_LOG_LEVEL_LOCAL(esp_log_level, "kvswebrtc", "%s: %s", tag, buffer);
  free(buffer);
}

esp_err_t webrtc_init() {
  globalCustomLogPrintFn = webrtc_logger;
  uint32_t status = initKvsWebRtc();
  if (status != STATUS_SUCCESS) {
    ESP_LOGE(RADIO_TAG,
             "Failed to initialize KVS WebRTC with status code %" PRIx32,
             status);
    return ESP_FAIL;
  }

  return ESP_OK;
}

// KVS WebRTC always uses mid=0 for trickle ICE so we can hardcode this
static const char *STREAM_MID_LINE = "a=mid:0";

static void webrtc_send_pending_candidates(webrtc_connection_t connection) {
  if (!connection->whep_client) {
    return;
  }

  char *sdp_fragment = NULL;

  xSemaphoreTake(connection->lock, portMAX_DELAY);

  size_t fragment_len =
      strlen(connection->local_ice_ufrag) + strlen(connection->local_ice_pwd) +
      strlen(connection->local_media_line) + strlen(STREAM_MID_LINE) + 8;
  int num_pending_candidates = 0;

  for (int i = 0; i < sizeof(connection->pending_candidates) /
                          sizeof(connection->pending_candidates[0]);
       i++) {
    if (connection->pending_candidates[i]) {
      fragment_len +=
          strlen("a=") + strlen(connection->pending_candidates[i]) + 2;
      num_pending_candidates++;
    } else {
      break;
    }
  }

  if (num_pending_candidates == 0) {
    goto cleanup;
  }

  fragment_len++; // for null terminator

  sdp_fragment = calloc(1, fragment_len);
  if (!sdp_fragment) {
    ESP_LOGE(RADIO_TAG, "Failed to allocate memory for SDP fragment");
    goto cleanup;
  }

  snprintf(sdp_fragment, fragment_len, "%s\r\n%s\r\n%s\r\n%s\r\n",
           connection->local_ice_ufrag, connection->local_ice_pwd,
           connection->local_media_line, STREAM_MID_LINE);

  for (int i = 0; i < num_pending_candidates; i++) {
    strlcat(sdp_fragment, "a=", fragment_len);
    strlcat(sdp_fragment, connection->pending_candidates[i], fragment_len);
    strlcat(sdp_fragment, "\r\n", fragment_len);
  }

  ESP_LOGV(RADIO_TAG, "Sending %d trickle ICE candidates to %s",
           num_pending_candidates, connection->session_url);
  esp_http_client_set_method(connection->whep_client, HTTP_METHOD_PATCH);
  esp_http_client_set_header(connection->whep_client, "Content-Type",
                             "application/trickle-ice-sdpfrag");
  esp_http_client_set_post_field(connection->whep_client, sdp_fragment,
                                 strlen(sdp_fragment));
  esp_err_t err = esp_http_client_perform(connection->whep_client);
  if (err != ESP_OK) {
    ESP_LOGE(RADIO_TAG, "Failed to send ICE candidates with error code %d (%s)",
             err, esp_err_to_name(err));
    goto cleanup;
  }
  ESP_LOGV(RADIO_TAG, "Received HTTP status from trickle ICE request: %d",
           esp_http_client_get_status_code(connection->whep_client));
  if (esp_http_client_get_status_code(connection->whep_client) >= 400) {
    ESP_LOGE(RADIO_TAG, "Failed to send ICE candidates with status code %d",
             esp_http_client_get_status_code(connection->whep_client));
    goto cleanup;
  }

  // Clear pending candidates
  for (int i = 0; i < num_pending_candidates; i++) {
    free(connection->pending_candidates[i]);
    connection->pending_candidates[i] = NULL;
  }

cleanup:
  free(sdp_fragment);

  xSemaphoreGive(connection->lock);
}

static void webrtc_on_connection_state_change(UINT64 arg,
                                              RTC_PEER_CONNECTION_STATE state) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
  webrtc_connection_t connection = (webrtc_connection_t)arg;
#pragma GCC diagnostic pop

  webrtc_connection_state_t new_state;
  switch (state) {
  case RTC_PEER_CONNECTION_STATE_NEW:
    new_state = WEBRTC_CONNECTION_STATE_NEW;
    break;
  case RTC_PEER_CONNECTION_STATE_CONNECTING:
    new_state = WEBRTC_CONNECTION_STATE_CONNECTING;
    break;
  case RTC_PEER_CONNECTION_STATE_CONNECTED:
    new_state = WEBRTC_CONNECTION_STATE_CONNECTED;
    break;
  case RTC_PEER_CONNECTION_STATE_DISCONNECTED:
    new_state = WEBRTC_CONNECTION_STATE_DISCONNECTED;
    break;
  case RTC_PEER_CONNECTION_STATE_FAILED:
    new_state = WEBRTC_CONNECTION_STATE_FAILED;
    break;
  case RTC_PEER_CONNECTION_STATE_CLOSED:
    new_state = WEBRTC_CONNECTION_STATE_CLOSED;
    break;
  default:
    ESP_LOGW(RADIO_TAG, "Unknown connection state %d", state);
    return;
  }

  connection->state = new_state;

  if (connection->state_change_callback) {
    connection->state_change_callback(connection, connection->user_data,
                                      new_state);
  }
}

static void webrtc_on_buffer_duration(UINT64 arg, UINT32 duration) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
  webrtc_connection_t connection = (webrtc_connection_t)arg;
#pragma GCC diagnostic pop

  if (connection->buffer_duration_callback) {
    connection->buffer_duration_callback(connection, connection->user_data,
                                         duration);
  }
}

static void webrtc_on_ice_candidate(UINT64 arg, PCHAR candidate) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
  webrtc_connection_t connection = (webrtc_connection_t)arg;
#pragma GCC diagnostic pop

  if (candidate == NULL) {
    // All candidates have received
    webrtc_send_pending_candidates(connection);
    return;
  }

  // Need to extract the candidate from the JSON-encoded string
  // (`{"candidate":"...","sdpMid":"0","sdpMLineIndex":0}`)
  jsmn_parser parser;
  jsmn_init(&parser);

  jsmntok_t tokens[7];
  int ret = jsmn_parse(&parser, candidate, strlen(candidate), tokens,
                       sizeof(tokens) / sizeof(tokens[0]));
  if (ret <= 0) {
    ESP_LOGE(RADIO_TAG, "Failed to parse ICE candidate JSON with error code %d",
             ret);
    return;
  }

  if (tokens[0].type != JSMN_OBJECT) {
    ESP_LOGE(RADIO_TAG, "Expected JSON object for ICE candidate, got %d",
             tokens[0].type);
    return;
  }

  char *candidate_start = NULL;
  char *candidate_end = NULL;
  for (int i = 1; i < ret - 1; i++) {
    if (tokens[i].type == JSMN_STRING &&
        strncmp(candidate + tokens[i].start, "candidate",
                tokens[i].end - tokens[i].start) == 0) {
      i++;
      if (i >= ret || tokens[i].type != JSMN_STRING) {
        ESP_LOGE(RADIO_TAG, "Expected string for ICE candidate, got %d",
                 tokens[i].type);
        return;
      }

      candidate_start = candidate + tokens[i].start;
      candidate_end = candidate + tokens[i].end;
      break;
    }
  }

  xSemaphoreTake(connection->lock, portMAX_DELAY);

  for (int i = 0; i < sizeof(connection->pending_candidates) /
                          sizeof(connection->pending_candidates[0]);
       i++) {
    if (!connection->pending_candidates[i]) {
      connection->pending_candidates[i] =
          strndup(candidate_start, candidate_end - candidate_start);
      break;
    }
  }

  xSemaphoreGive(connection->lock);

  webrtc_send_pending_candidates(connection);
}

static esp_err_t
webrtc_connect_http_event_handler(esp_http_client_event_t *evt) {
  struct webrtc_connect_http_data *data =
      (struct webrtc_connect_http_data *)evt->user_data;

  if (evt->event_id == HTTP_EVENT_ON_DATA) {
    if (strlen(data->remoteDescription->sdp) + evt->data_len + 1 >=
        MAX_SESSION_DESCRIPTION_INIT_SDP_LEN) {
      ESP_LOGE(RADIO_TAG, "SDP offer too large");
      return ESP_FAIL;
    }
    strncat(data->remoteDescription->sdp, evt->data, evt->data_len);
  } else if (evt->event_id == HTTP_EVENT_ON_HEADER) {
    if (strcasecmp(evt->header_key, "Location") == 0) {
      data->connection->session_url = strdup(evt->header_value);
    } else if (strcasecmp(evt->header_key, "Link") == 0) {
      // TODO: Check for rel="ice-server" and then parse out additional
      // STUN/TURN servers to pass to addConfigToServerList
      //
      // Header looks like:
      //
      // Link: <stun:stun.example.net>; rel="ice-server"
      // Link: <turn:turn.example.net?transport=udp>; rel="ice-server";
      //       username="user"; credential="myPassword";
      //       credential-type="password"
    }
  }
  return ESP_OK;
}

esp_err_t webrtc_connect(webrtc_config_t *config, webrtc_connection_t *handle) {
  ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, RADIO_TAG, "Invalid config");
  ESP_RETURN_ON_FALSE(config->whep_url, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Invalid WHEP URL");
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, RADIO_TAG, "Invalid handle");

  esp_err_t ret = ESP_OK;

  PRtcConfiguration rtc_cfg = NULL;
  PRtcMediaStreamTrack track = NULL;
  PRtcSessionDescriptionInit offer = NULL;
  esp_http_client_handle_t http_client = NULL;
  PRtcSessionDescriptionInit remoteDescription = NULL;

  webrtc_connection_t connection = calloc(1, sizeof(struct webrtc_connection));
  ESP_GOTO_ON_FALSE(connection, ESP_ERR_NO_MEM, cleanup, RADIO_TAG,
                    "Failed to allocate memory for webrtc connection");
  *handle = connection;
  connection->state_change_callback = config->state_change_callback;
  connection->buffer_duration_callback = config->buffer_duration_callback;
  connection->user_data = config->user_data;

  int error;
  connection->decoder = opus_decoder_create(48000, 1, &error);
  ESP_GOTO_ON_FALSE(error == OPUS_OK, ESP_FAIL, cleanup, RADIO_TAG,
                    "Failed to create Opus decoder with error code %s",
                    opus_strerror(error));

  connection->lock = xSemaphoreCreateMutex();

  rtc_cfg = calloc(1, sizeof(RtcConfiguration));
  ESP_GOTO_ON_FALSE(rtc_cfg, ESP_ERR_NO_MEM, cleanup, RADIO_TAG,
                    "Failed to allocate memory for RTC configuration");
  rtc_cfg->iceTransportPolicy = ICE_TRANSPORT_POLICY_ALL;
  strlcpy(rtc_cfg->iceServers[0].urls, "stun:stun.l.google.com:19302",
          sizeof(rtc_cfg->iceServers[0].urls));
  rtc_cfg->kvsRtcConfiguration.disableSenderSideBandwidthEstimation = TRUE;

  STATUS status = createPeerConnection(rtc_cfg, &connection->peer_connection);
  ESP_GOTO_ON_FALSE(
      status == STATUS_SUCCESS, ESP_FAIL, cleanup, RADIO_TAG,
      "Failed to create peer connection with status code %" PRIx32, status);

  track = calloc(1, sizeof(RtcMediaStreamTrack));
  ESP_GOTO_ON_FALSE(track, ESP_ERR_NO_MEM, cleanup, RADIO_TAG,
                    "Failed to allocate memory for media webrtc track");
  track->kind = MEDIA_STREAM_TRACK_KIND_AUDIO;
  track->codec = RTC_CODEC_OPUS;
  strlcpy(track->streamId, "audio", sizeof(track->streamId));
  strlcpy(track->trackId, "audio", sizeof(track->trackId));
  RtcRtpTransceiverInit transceiverInit = {
      .direction = RTC_RTP_TRANSCEIVER_DIRECTION_RECVONLY,
  };

  status = addTransceiver(connection->peer_connection, track, &transceiverInit,
                          &connection->transceiver);
  ESP_GOTO_ON_FALSE(status == STATUS_SUCCESS, ESP_FAIL, cleanup, RADIO_TAG,
                    "Failed to add transceiver with status code %" PRIx32,
                    status);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpointer-to-int-cast"
  peerConnectionOnIceCandidate(connection->peer_connection, (UINT64)connection,
                               webrtc_on_ice_candidate);
  peerConnectionOnConnectionStateChange(connection->peer_connection,
                                        (UINT64)connection,
                                        webrtc_on_connection_state_change);
  transceiverOnBufferDuration(connection->transceiver, (UINT64)connection,
                              webrtc_on_buffer_duration);
#pragma GCC diagnostic pop

  offer = calloc(1, sizeof(RtcSessionDescriptionInit));
  ESP_GOTO_ON_FALSE(offer, ESP_ERR_NO_MEM, cleanup, RADIO_TAG,
                    "Failed to allocate memory for SDP offer");
  offer->useTrickleIce = TRUE;
  status = createOffer(connection->peer_connection, offer);
  ESP_GOTO_ON_FALSE(status == STATUS_SUCCESS, ESP_FAIL, cleanup, RADIO_TAG,
                    "Failed to create offer with status code %" PRIx32, status);

  // capture m=, a=ice-ufrag:, and a=ice-pwd: lines from offer for trickle ICE
  char *sdp = offer->sdp;
  char *media_line = strstr(sdp, "m=");
  ESP_GOTO_ON_FALSE(media_line, ESP_FAIL, cleanup, RADIO_TAG,
                    "Failed to find media line in SDP offer");
  connection->local_media_line =
      strndup(media_line, strcspn(media_line, "\r\n"));

  char *ice_ufrag = strstr(sdp, "a=ice-ufrag:");
  ESP_GOTO_ON_FALSE(ice_ufrag, ESP_FAIL, cleanup, RADIO_TAG,
                    "Failed to find ICE ufrag in SDP offer");
  connection->local_ice_ufrag = strndup(ice_ufrag, strcspn(ice_ufrag, "\r\n"));

  char *ice_pwd = strstr(sdp, "a=ice-pwd:");
  ESP_GOTO_ON_FALSE(ice_pwd, ESP_FAIL, cleanup, RADIO_TAG,
                    "Failed to find ICE pwd in SDP offer");
  connection->local_ice_pwd = strndup(ice_pwd, strcspn(ice_pwd, "\r\n"));

  status = setLocalDescription(connection->peer_connection, offer);
  ESP_GOTO_ON_FALSE(status == STATUS_SUCCESS, ESP_FAIL, cleanup, RADIO_TAG,
                    "Failed to set local description with status code %" PRIx32,
                    status);

  // Send the offer to the WHEP server
  remoteDescription = calloc(1, SIZEOF(RtcSessionDescriptionInit));
  ESP_GOTO_ON_FALSE(remoteDescription, ESP_ERR_NO_MEM, cleanup, RADIO_TAG,
                    "Failed to allocate memory for remote description");

  struct webrtc_connect_http_data http_data = {
      .connection = connection,
      .remoteDescription = remoteDescription,
  };
  esp_http_client_config_t http_config = {
      .url = config->whep_url,
      .method = HTTP_METHOD_POST,
      .transport_type = strcasecmp("https://", config->whep_url) == 0
                            ? HTTP_TRANSPORT_OVER_SSL
                            : HTTP_TRANSPORT_OVER_TCP,
      .user_data = &http_data,
      .event_handler = webrtc_connect_http_event_handler,
      .crt_bundle_attach = esp_crt_bundle_attach,
  };
  http_client = esp_http_client_init(&http_config);
  ESP_GOTO_ON_FALSE(http_client, ESP_FAIL, cleanup, RADIO_TAG,
                    "Failed to initialize HTTP client");
  esp_http_client_set_header(http_client, "Content-Type", "application/sdp");
  esp_http_client_set_post_field(http_client, offer->sdp, strlen(offer->sdp));
  ESP_GOTO_ON_ERROR(esp_http_client_perform(http_client), cleanup, RADIO_TAG,
                    "Failed to send SDP offer with error code %d (%s)", err_rc_,
                    esp_err_to_name(err_rc_));
  ESP_LOGV(RADIO_TAG, "Received HTTP status from SDP offer request: %d",
           esp_http_client_get_status_code(http_client));
  ESP_GOTO_ON_FALSE(esp_http_client_get_status_code(http_client) < 400,
                    ESP_FAIL, cleanup, RADIO_TAG,
                    "Failed to send SDP offer with status code %d",
                    esp_http_client_get_status_code(http_client));

  status = setRemoteDescription(connection->peer_connection, remoteDescription);
  ESP_GOTO_ON_FALSE(
      status == STATUS_SUCCESS, ESP_FAIL, cleanup, RADIO_TAG,
      "Failed to set remote description with status code %" PRIx32, status);

  esp_http_client_set_url(http_client, connection->session_url);
  connection->whep_client = http_client;
  http_client = NULL;

  // Flush any pending ICE candidates that came in while we were waiting for the
  // session to be established
  webrtc_send_pending_candidates(connection);

cleanup:
  esp_http_client_cleanup(http_client);
  free(remoteDescription);
  free(offer);
  free(track);
  free(rtc_cfg);

  if (ret != ESP_OK) {
    webrtc_free_connection(connection);
    *handle = NULL;
  }

  return ret;
}

webrtc_connection_state_t webrtc_get_state(webrtc_connection_t connection) {
  return connection ? connection->state : WEBRTC_CONNECTION_STATE_NONE;
}

const char *webrtc_connection_state_to_string(webrtc_connection_state_t state) {
  switch (state) {
  case WEBRTC_CONNECTION_STATE_NONE:
    return "None";
  case WEBRTC_CONNECTION_STATE_NEW:
    return "New";
  case WEBRTC_CONNECTION_STATE_CONNECTING:
    return "Connecting";
  case WEBRTC_CONNECTION_STATE_CONNECTED:
    return "Connected";
  case WEBRTC_CONNECTION_STATE_DISCONNECTED:
    return "Disconnected";
  case WEBRTC_CONNECTION_STATE_FAILED:
    return "Failed";
  case WEBRTC_CONNECTION_STATE_CLOSED:
    return "Closed";
  default:
    return "Unknown";
  }
}

esp_err_t webrtc_get_buffer_duration(webrtc_connection_t connection,
                                     uint32_t *duration) {
  STATUS ret = transceiverGetBufferDuration(connection->transceiver, duration);
  if (ret != STATUS_SUCCESS) {
    ESP_LOGE(RADIO_TAG,
             "Failed to get buffer duration with status code %" PRIx32, ret);
    return ESP_FAIL;
  }

  return ESP_OK;
}

esp_err_t webrtc_wait_buffer_duration(webrtc_connection_t connection,
                                      uint32_t duration_samples,
                                      uint64_t max_wait_ms) {
  STATUS ret = transceiverWaitBufferDuration(
      connection->transceiver, duration_samples, max_wait_ms * 10);
  if (ret != STATUS_SUCCESS) {
    ESP_LOGE(RADIO_TAG,
             "Failed to wait for buffer duration with status code %" PRIx32,
             ret);
    return ESP_FAIL;
  }

  return ESP_OK;
}

struct webrtc_on_frame_data {
  webrtc_connection_t connection;
  char *buf;
  int buf_len;
};

static void webrtc_read_on_frame(UINT64 custom_data, PFrame frame) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
  struct webrtc_on_frame_data *data =
      (struct webrtc_on_frame_data *)custom_data;
#pragma GCC diagnostic pop

  int decoded = 0;
  size_t bytes_per_sample = sizeof(opus_int16) / sizeof(data->buf[0]);

  if (data->buf != NULL) {
    if (data->connection->last_received_packet_len == 0) {
      // Missing last packet.
      if (frame->size == 0) {
        // Missing current packet too. Trigger PLC
        ESP_LOGD(RADIO_TAG, "Previous two packets missing, triggering PLC");
        opus_int32 packet_duration;
        opus_decoder_ctl(data->connection->decoder,
                         OPUS_GET_LAST_PACKET_DURATION(&packet_duration));
        decoded = opus_decode(data->connection->decoder, NULL, 0,
                              (opus_int16 *)data->buf, packet_duration, 0);
      } else {
        // We have a new packet, so trigger FEC
        ESP_LOGV(RADIO_TAG, "Previous packet missing, triggering FEC");
        decoded = opus_decode(data->connection->decoder,
                              (unsigned char *)frame->frameData, frame->size,
                              (opus_int16 *)data->buf,
                              data->buf_len / bytes_per_sample, 1);
      }
    } else {
      // Decode the packet
      decoded = opus_decode(data->connection->decoder,
                            (unsigned char *)frame->frameData, frame->size,
                            (opus_int16 *)data->buf,
                            data->buf_len / bytes_per_sample, 0);
    }
  }

  if (decoded < 0) {
    ESP_LOGE(RADIO_TAG, "Failed to decode Opus frame with error code %d (%s)",
             decoded, opus_strerror(decoded));
    data->buf_len = -1;
    return;
  }

  if (data->connection->last_received_packet_capacity < frame->size) {
    data->connection->last_received_packet =
        realloc(data->connection->last_received_packet, frame->size);
    if (!data->connection->last_received_packet) {
      ESP_LOGE(RADIO_TAG, "Failed to allocate memory for last received packet");
      data->buf_len = -1;
      return;
    }
    data->connection->last_received_packet_capacity = frame->size;
  }
  memcpy(data->connection->last_received_packet, frame->frameData, frame->size);
  data->connection->last_received_packet_len = frame->size;

  data->buf_len = decoded * bytes_per_sample;
}

int webrtc_read_audio_sample(void *context, char *buf, int len,
                             TickType_t ticks_to_wait) {
  webrtc_connection_t connection = (webrtc_connection_t)context;
  if (!connection || connection->state != WEBRTC_CONNECTION_STATE_CONNECTED) {
    return -1;
  }

  struct webrtc_on_frame_data data = {
      .connection = connection,
      .buf = buf,
      .buf_len = len,
  };

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpointer-to-int-cast"
  transceiverPopFrame(connection->transceiver, (UINT64)&data,
                      webrtc_read_on_frame);
#pragma GCC diagnostic pop

  return data.buf_len;
}

float webrtc_get_ice_rtt_ms(webrtc_connection_t connection) {
  PRtcStats stats = calloc(1, sizeof(RtcStats));
  if (!stats) {
    ESP_LOGE(RADIO_TAG, "Failed to allocate memory for RTC stats");
    return -1;
  }

  int ret = -1;

  stats->requestedTypeOfStats = RTC_STATS_TYPE_ICE_SERVER;
  STATUS status =
      rtcPeerConnectionGetMetrics(connection->peer_connection, NULL, stats);
  if (status != STATUS_SUCCESS) {
    ESP_LOGE(RADIO_TAG, "Failed to get ICE metrics with status code %" PRIx32,
             status);
    ret = -1;
    goto cleanup;
  }
  if (stats->rtcStatsObject.iceServerStats.totalRequestsSent == 0) {
    ret = -1;
    goto cleanup;
  }
  ret = (float)stats->rtcStatsObject.iceServerStats.totalRoundTripTime /
        stats->rtcStatsObject.iceServerStats.totalRequestsSent;
  ret /= HUNDREDS_OF_NANOS_IN_A_MILLISECOND;
cleanup:
  free(stats);
  return ret;
}