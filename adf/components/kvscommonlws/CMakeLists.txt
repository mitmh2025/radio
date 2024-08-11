cmake_minimum_required(VERSION 3.12)

idf_component_register(REQUIRES kvspic mbedtls lws)

idf_component_get_property(KVSPIC_PREFIX kvspic KVSPIC_PREFIX)

set(KVSCOMMONLWS_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/kvscommonlws_install)
idf_component_set_property(${COMPONENT_NAME} KVSCOMMONLWS_PREFIX ${KVSCOMMONLWS_INSTALL_DIR})

include(ExternalProject)

ExternalProject_Add(kvscommonlws
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/amazon-kinesis-video-streams-producer-c
  CMAKE_ARGS
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    -DBUILD_STATIC=ON
    -DBUILD_DEPENDENCIES=OFF
    -DUSE_OPENSSL=OFF
    -DUSE_MBEDTLS=ON
    -DBUILD_COMMON_LWS=ON
    -DBUILD_COMMON_CURL=OFF
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    -DCMAKE_PROJECT_KinesisVideoProducerC_INCLUDE=${CMAKE_SOURCE_DIR}/cmake/add-extra-properties.cmake
    -DEXTRA_INCLUDE_DIRECTORIES=$<TARGET_PROPERTY:${COMPONENT_TARGET},INTERFACE_INCLUDE_DIRECTORIES>
    -DCMAKE_PREFIX_PATH=${KVSPIC_PREFIX}
    -DCMAKE_INCLUDE_PATH=$<TARGET_PROPERTY:${COMPONENT_TARGET},INTERFACE_INCLUDE_DIRECTORIES>
    -DCMAKE_LIBRARY_PATH=$<TARGET_FILE_DIR:mbedtls>
  CMAKE_CACHE_ARGS
    -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
  INSTALL_DIR ${KVSCOMMONLWS_INSTALL_DIR}
  BUILD_BYPRODUCTS
    ${KVSCOMMONLWS_INSTALL_DIR}/lib/libkvsCommonLws.a
  DEPENDS kvspic idf::lws
)

add_prebuilt_library(kvscommonlws_lib "${KVSCOMMONLWS_INSTALL_DIR}/lib/libkvsCommonLws.a")
target_include_directories(kvscommonlws_lib INTERFACE "${KVSCOMMONLWS_INSTALL_DIR}/include")
add_dependencies(kvscommonlws_lib kvscommonlws)

target_link_libraries(${COMPONENT_LIB} INTERFACE kvscommonlws_lib)
