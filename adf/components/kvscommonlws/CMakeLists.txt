cmake_minimum_required(VERSION 3.12)

idf_component_register(REQUIRES kvspic mbedtls lws)

idf_component_get_property(KVSPIC_PREFIX kvspic KVSPIC_PREFIX)

set(KVSCOMMONLWS_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/kvscommonlws_install)
idf_component_set_property(${COMPONENT_NAME} KVSCOMMONLWS_PREFIX ${KVSCOMMONLWS_INSTALL_DIR})

include(ExternalProject)

ExternalProject_Add(kvscommonlws
  GIT_REPOSITORY https://github.com/awslabs/amazon-kinesis-video-streams-producer-c
  GIT_TAG 3e519b7670e39031375d227f983ad2cde888078e
  PATCH_COMMAND sed -i "" -e "/LIBWEBSOCKETS/d" CMakeLists.txt
  CMAKE_ARGS
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    -DBUILD_STATIC=ON
    -DBUILD_DEPENDENCIES=OFF
    -DUSE_OPENSSL=OFF
    -DUSE_MBEDTLS=ON
    -DBUILD_COMMON_LWS=ON
    -DBUILD_COMMON_CURL=OFF
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    -DCMAKE_PROJECT_KinesisVideoProducerC_INCLUDE=${CMAKE_SOURCE_DIR}/cmake/add-extra-include-directories.cmake
    -DEXTRA_INCLUDE_DIRECTORIES=$<TARGET_PROPERTY:${COMPONENT_TARGET},INTERFACE_INCLUDE_DIRECTORIES>
    -DCMAKE_PREFIX_PATH=${KVSPIC_PREFIX}
    -DCMAKE_INCLUDE_PATH=$<TARGET_PROPERTY:${COMPONENT_TARGET},INTERFACE_INCLUDE_DIRECTORIES>
    -DCMAKE_LIBRARY_PATH=$<TARGET_FILE_DIR:mbedtls>
  INSTALL_DIR ${KVSCOMMONLWS_INSTALL_DIR}
  BUILD_BYPRODUCTS
    ${KVSCOMMONLWS_INSTALL_DIR}/lib/libkvsCommonLws.a
  DEPENDS kvspic idf::lws
)

add_prebuilt_library(kvscommonlws_lib "${KVSCOMMONLWS_INSTALL_DIR}/lib/libkvsCommonLws.a")
target_include_directories(kvscommonlws_lib INTERFACE "${KVSCOMMONLWS_INSTALL_DIR}/include")
add_dependencies(kvscommonlws_lib kvscommonlws)

target_link_libraries(${COMPONENT_LIB} INTERFACE kvscommonlws_lib)
