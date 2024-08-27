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

  // WEBRTC_CONNECTION_STATE is taken from kvswebrtcclient
  typedef enum {
      WEBRTC_CONNECTION_STATE_NONE = 0,         //!< Starting state of peer connection
      WEBRTC_CONNECTION_STATE_NEW = 1,          //!< This state is set when ICE Agent is waiting for remote credential
      WEBRTC_CONNECTION_STATE_CONNECTING = 2,   //!< This state is set when ICE agent checks connection
      WEBRTC_CONNECTION_STATE_CONNECTED = 3,    //!< This state is set when CIE Agent is ready
      WEBRTC_CONNECTION_STATE_DISCONNECTED = 4, //!< This state is set when ICE Agent is disconnected
      WEBRTC_CONNECTION_STATE_FAILED = 5,       //!< This state is set when ICE Agent transitions to fail state
      WEBRTC_CONNECTION_STATE_CLOSED = 6,       //!< This state leads to termination of streaming session
      WEBRTC_CONNECTION_TOTAL_STATE_COUNT = 7,  //!< This state indicates maximum number of Peer connection states
  } WEBRTC_CONNECTION_STATE;

  // struct webrtc_connection is opaque because we need to reference types from
  // kvswebrtcclient
  typedef struct webrtc_connection *webrtc_connection_t;

  typedef void (*webrtc_connection_state_change_callback_t)(webrtc_connection_t conn, void *user_data, WEBRTC_CONNECTION_STATE state);

  typedef struct {
    const char *whep_url;
    webrtc_connection_state_change_callback_t state_change_callback;
    void *user_data;
  } webrtc_config_t;

  esp_err_t webrtc_init();
  esp_err_t webrtc_connect(webrtc_config_t *cfg, webrtc_connection_t *connection);
  void webrtc_free_connection(webrtc_connection_t connection);
  audio_element_err_t webrtc_read_audio_sample(audio_element_handle_t el, char *buf, int len, TickType_t ticks_to_wait, void *context);

#ifdef __cplusplus
}
#endif
