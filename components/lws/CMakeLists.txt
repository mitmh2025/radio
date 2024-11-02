idf_component_register(REQUIRES driver esp_wifi nvs_flash PRIV_REQUIRES esp_driver_gpio)

set(LWS_WITH_DRIVERS OFF)
set(LWS_WITHOUT_SERVER ON)
set(LWS_WITHOUT_TEST_SERVER ON)
set(LWS_WITHOUT_TEST_SERVER_EXTPOLL ON)
set(LWS_WITHOUT_TEST_PING ON)
set(LWS_WITHOUT_TEST_CLIENT ON)
set(LWS_WITH_EXPORT_LWSTARGETS OFF)
set(DISABLE_WERROR ON)
include_directories($ENV{IDF_PATH}/components/esp_driver_gpio/include)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
add_subdirectory(libwebsockets)

set_directory_properties(PROPERTIES ADDITIONAL_CLEAN_FILES "${CMAKE_CURRENT_BINARY_DIR}/libwebsockets/include")

target_link_libraries(${COMPONENT_LIB} INTERFACE websockets)
