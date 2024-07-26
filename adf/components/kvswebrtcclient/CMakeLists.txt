cmake_minimum_required(VERSION 3.12)

idf_component_register(REQUIRES srtp libcextras kvscommonlws kvspic mbedtls lws)

idf_component_get_property(KVSCOMMONLWS_PREFIX kvscommonlws KVSCOMMONLWS_PREFIX)
idf_component_get_property(SRTP_PREFIX srtp SRTP_PREFIX)

idf_component_get_property(libcextras_lib libcextras COMPONENT_LIB)

set(KVSWEBRTCCLIENT_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/kvswebrtcclient_install)

include(ExternalProject)

ExternalProject_Add(kvswebrtcclient
  GIT_REPOSITORY git@github.com:mitmh2025/amazon-kinesis-video-streams-webrtc-sdk-c
  GIT_TAG 4f6eabcc9d58c1e85a5fd2cc3a22e25567e76550
  CMAKE_ARGS
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    -DBUILD_STATIC_LIBS=ON
    -DBUILD_DEPENDENCIES=OFF
    -DUSE_OPENSSL=OFF
    -DUSE_MBEDTLS=ON
    -DBUILD_SAMPLE=OFF
    -DENABLE_DATA_CHANNEL=OFF
    -DENABLE_KVS_THREADPOOL=OFF
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    -DCMAKE_PROJECT_KinesisVideoWebRTCClient_INCLUDE=${CMAKE_SOURCE_DIR}/cmake/add-extra-properties.cmake
    -DEXTRA_INCLUDE_DIRECTORIES=$<TARGET_PROPERTY:${libcextras_lib},INTERFACE_INCLUDE_DIRECTORIES>
    -DEXTRA_COMPILE_DEFINITIONS=$<TARGET_PROPERTY:${COMPONENT_TARGET},INTERFACE_COMPILE_DEFINITIONS>
    -DMBEDTLS_INCLUDE_DIRS=$<TARGET_PROPERTY:${COMPONENT_TARGET},INTERFACE_INCLUDE_DIRECTORIES>
    -DMBEDTLS_LIBRARY=$<TARGET_FILE:mbedtls>
    -DMBEDX509_LIBRARY=$<TARGET_FILE:mbedx509>
    -DMBEDCRYPTO_LIBRARY=$<TARGET_FILE:mbedcrypto>
  CMAKE_CACHE_ARGS
    -DKVSWEBRTC_HAVE_IFADDRS_H:BOOL=ON
    -DKVSWEBRTC_HAVE_GETIFADDRS:BOOL=ON
    -DCMAKE_PREFIX_PATH:STRING=${KVSCOMMONLWS_PREFIX}\\;${SRTP_PREFIX}
  INSTALL_DIR ${KVSWEBRTCCLIENT_INSTALL_DIR}
  BUILD_BYPRODUCTS
    ${KVSWEBRTCCLIENT_INSTALL_DIR}/lib/libkvsWebrtcClient.a
  DEPENDS kvspic srtp kvscommonlws idf::libcextras idf::lws
)

add_prebuilt_library(kvswebrtcclient_lib "${KVSWEBRTCCLIENT_INSTALL_DIR}/lib/libkvsWebrtcClient.a" REQUIRES srtp mbedtls libcextras kvspic)
target_include_directories(kvswebrtcclient_lib INTERFACE "${KVSWEBRTCCLIENT_INSTALL_DIR}/include")
add_dependencies(kvswebrtcclient_lib kvswebrtcclient)

target_link_libraries(${COMPONENT_LIB} INTERFACE kvswebrtcclient_lib)
