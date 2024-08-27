FILE(GLOB_RECURSE component_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.c)

idf_component_register(SRCS ${component_sources} INCLUDE_DIRS include REQUIRES esp_netif lwip)

idf_component_get_property(lwip_component lwip COMPONENT_LIB)
