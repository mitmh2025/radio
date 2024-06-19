idf_component_register(SRCS "wifi.c" "main.c"
                       INCLUDE_DIRS "."
                       EMBED_FILES "big-country.mp3"
                       PRIV_REQUIRES nvs_flash esp_event esp_wifi audio_pipeline audio_stream)
