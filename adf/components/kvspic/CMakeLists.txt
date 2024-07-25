cmake_minimum_required(VERSION 3.12)

idf_component_register(REQUIRES uname)

foreach(LIB IN ITEMS newlib uname freertos xtensa esp_hw_support soc esp_common heap esp_rom esp_system)
  idf_component_get_property(tmp_lib ${LIB} COMPONENT_LIB)
  get_target_property(tmp_includes ${tmp_lib} INTERFACE_INCLUDE_DIRECTORIES)
  list(APPEND INCLUDES ${tmp_includes})
endforeach()

list(TRANSFORM INCLUDES PREPEND "-I")
list(JOIN INCLUDES " " KVSPIC_CFLAGS)

set(KVSPIC_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/kvspic_install)

include(ExternalProject)

ExternalProject_Add(kvspic
  GIT_REPOSITORY git@github.com:mitmh2025/amazon-kinesis-video-streams-pic
  GIT_TAG e89f6e2984aec154ae49d9e2b79b000511bab511
  CMAKE_ARGS
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    -DBUILD_DEPENDENCIES=OFF
    -DBUILD_TEST=OFF
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    -DCMAKE_C_FLAGS=${KVSPIC_CFLAGS}
    -DCMAKE_CXX_FLAGS=${KVSPIC_CFLAGS}
  INSTALL_DIR ${KVSPIC_INSTALL_DIR}
  BUILD_BYPRODUCTS
    ${KVSPIC_INSTALL_DIR}/lib/libkvspic.a
    ${KVSPIC_INSTALL_DIR}/lib/libkvspicClient.a
    ${KVSPIC_INSTALL_DIR}/lib/libkvspicState.a
    ${KVSPIC_INSTALL_DIR}/lib/libkvspicUtils.a
)

add_prebuilt_library(kvspic_lib "${KVSPIC_INSTALL_DIR}/lib/libkvspic.a")
target_include_directories(kvspic_lib INTERFACE "${KVSPIC_INSTALL_DIR}/include")
add_dependencies(kvspic_lib kvspic)

add_prebuilt_library(kvspicclient_lib "${KVSPIC_INSTALL_DIR}/lib/libkvspicClient.a")
target_include_directories(kvspicclient_lib INTERFACE "${KVSPIC_INSTALL_DIR}/include")
add_dependencies(kvspicclient_lib kvspic)

add_prebuilt_library(kvspicstate_lib "${KVSPIC_INSTALL_DIR}/lib/libkvspicState.a")
target_include_directories(kvspicstate_lib INTERFACE "${KVSPIC_INSTALL_DIR}/include")
add_dependencies(kvspicstate_lib kvspic)

add_prebuilt_library(kvspicutils_lib "${KVSPIC_INSTALL_DIR}/lib/libkvspicUtils.a")
target_include_directories(kvspicutils_lib INTERFACE "${KVSPIC_INSTALL_DIR}/include")
add_dependencies(kvspicutils_lib kvspic)

target_link_libraries(${COMPONENT_LIB} INTERFACE kvspic_lib)
target_link_libraries(${COMPONENT_LIB} INTERFACE kvspicclient_lib)
target_link_libraries(${COMPONENT_LIB} INTERFACE kvspicstate_lib)
target_link_libraries(${COMPONENT_LIB} INTERFACE kvspicutils_lib)
