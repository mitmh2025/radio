FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/main/*.*)

idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "."
                       EMBED_FILES "song.mp3"
                       PRIV_REQUIRES nvs_flash esp_event esp_wifi audio_pipeline audio_stream)
