#pragma once

#include "esp_err.h"

// The KVS WebRTC client has types which conflict with some ESP-IDF types (in
// particular, STATUS), which means we can't include the header in a file which
// will be included in the main entry point.

#ifdef __cplusplus
extern "C"
{
#endif

  // STREAM_CONNECTION_STATE is taken from kvswebrtcclient
  typedef enum {
      STREAM_CONNECTION_STATE_NONE = 0,         //!< Starting state of peer connection
      STREAM_CONNECTION_STATE_NEW = 1,          //!< This state is set when ICE Agent is waiting for remote credential
      STREAM_CONNECTION_STATE_CONNECTING = 2,   //!< This state is set when ICE agent checks connection
      STREAM_CONNECTION_STATE_CONNECTED = 3,    //!< This state is set when CIE Agent is ready
      STREAM_CONNECTION_STATE_DISCONNECTED = 4, //!< This state is set when ICE Agent is disconnected
      STREAM_CONNECTION_STATE_FAILED = 5,       //!< This state is set when ICE Agent transitions to fail state
      STREAM_CONNECTION_STATE_CLOSED = 6,       //!< This state leads to termination of streaming session
      STREAM_CONNECTION_TOTAL_STATE_COUNT = 7,  //!< This state indicates maximum number of Peer connection states
  } STREAM_CONNECTION_STATE;

  typedef void (*stream_connection_state_change_callback_t)(void *user_data, STREAM_CONNECTION_STATE state);

  typedef struct {
    const char *whep_url;
    stream_connection_state_change_callback_t state_change_callback;
    void *user_data;
  } stream_config_t;

  // struct stream_connection is opaque because we need to reference types from
  // kvswebrtcclient
  typedef struct stream_connection *stream_connection_t;

  esp_err_t stream_init();
  esp_err_t stream_connect(stream_config_t *cfg, stream_connection_t *connection);
  void stream_free_connection(stream_connection_t connection);

#ifdef __cplusplus
}
#endif
