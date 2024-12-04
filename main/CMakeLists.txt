idf_component_register(
  SRCS
  accelerometer.c
  adc.c
  audio_output.c
  audio_volume.c
  battery.c
  bluetooth.c
  bounds.c
  calibration.c
  captive_dns_server.c
  captive_http_server.c
  console.c
  debounce.c
  file_cache.c
  fm.c
  led.c
  magnet.c
  main.c
  mixer.c
  playback.c
  static.c
  station_pi.c
  station_pi_activation.c
  station_2pi.c
  station_wifi.c
  storage.c
  tas2505.c
  things.cpp
  tone_generator.c
  touch.c
  tuner.c
  webrtc.c
  webrtc_manager.c
  wifi.c
  INCLUDE_DIRS
  "."
  REQUIRES
  audio_pipeline
  esp_adc
  PRIV_REQUIRES
  app_update
  audio_board
  audio_stream
  boardconfig
  bt
  console
  file_stream
  esp_app_format
  esp_http_client
  esp_http_server
  esp_littlefs
  espcoredump
  kvscommonlws
  kvswebrtcclient
  opus
  opusfile
  sdmmc)

foreach(FILE IN ITEMS index.html style.css)
  add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/captive/${FILE}.gz"
                    COMMAND gzip -c -9 "${CMAKE_CURRENT_SOURCE_DIR}/captive/${FILE}" > "${CMAKE_CURRENT_BINARY_DIR}/captive/${FILE}.gz"
                    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/captive/${FILE}"
                    VERBATIM)
  set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_CURRENT_BINARY_DIR}/captive/${FILE}.gz")
  target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_BINARY_DIR}/captive/${FILE}.gz" BINARY)
endforeach()

target_compile_options(${COMPONENT_LIB} PRIVATE -Werror)
