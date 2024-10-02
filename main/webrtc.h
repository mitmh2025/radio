#pragma once

#include "esp_err.h"

#include "audio_element.h"

// The KVS WebRTC client has types which conflict with some ESP-IDF types (in
// particular, STATUS), which means we can't include the header in a file which
// will be included in the main entry point.

#ifdef __cplusplus
extern "C"
{
#endif

  // webrtc_connection_state_t is taken from kvswebrtcclient
  typedef enum
  {
    WEBRTC_CONNECTION_STATE_NONE = 0,         //!< Starting state of peer connection
    WEBRTC_CONNECTION_STATE_NEW = 1,          //!< This state is set when ICE Agent is waiting for remote credential
    WEBRTC_CONNECTION_STATE_CONNECTING = 2,   //!< This state is set when ICE agent checks connection
    WEBRTC_CONNECTION_STATE_CONNECTED = 3,    //!< This state is set when CIE Agent is ready
    WEBRTC_CONNECTION_STATE_DISCONNECTED = 4, //!< This state is set when ICE Agent is disconnected
    WEBRTC_CONNECTION_STATE_FAILED = 5,       //!< This state is set when ICE Agent transitions to fail state
    WEBRTC_CONNECTION_STATE_CLOSED = 6,       //!< This state leads to termination of streaming session
    WEBRTC_CONNECTION_TOTAL_STATE_COUNT = 7,  //!< This state indicates maximum number of Peer connection states
  } webrtc_connection_state_t;

  // struct webrtc_connection is opaque because we need to reference types from
  // kvswebrtcclient
  typedef struct webrtc_connection *webrtc_connection_t;

  typedef void (*webrtc_connection_state_change_callback_t)(webrtc_connection_t conn, void *user_data, webrtc_connection_state_t state);

  typedef struct {
    const char *whep_url;
    webrtc_connection_state_change_callback_t state_change_callback;
    void *user_data;
  } webrtc_config_t;

  esp_err_t webrtc_init();
  esp_err_t webrtc_connect(webrtc_config_t *cfg, webrtc_connection_t *connection);
  void webrtc_free_connection(webrtc_connection_t connection);
  esp_err_t webrtc_get_buffer_duration(webrtc_connection_t connection, uint32_t *duration);
  esp_err_t webrtc_wait_buffer_duration(webrtc_connection_t connection, uint32_t duration_samples, uint64_t max_wait_ms);
  int webrtc_read_audio_sample(void *context, char *buf, int len, TickType_t ticks_to_wait);
  float webrtc_get_ice_rtt_ms(webrtc_connection_t connection);

#ifdef __cplusplus
}
#endif
