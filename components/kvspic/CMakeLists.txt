cmake_minimum_required(VERSION 3.12)

idf_component_register(REQUIRES libcextras)

set(KVSPIC_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/kvspic_install)
idf_component_set_property(${COMPONENT_NAME} KVSPIC_PREFIX ${KVSPIC_INSTALL_DIR})

include(ExternalProject)

ExternalProject_Add(kvspic
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/amazon-kinesis-video-streams-pic
  CMAKE_ARGS
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    -DBUILD_DEPENDENCIES=OFF
    -DBUILD_TEST=OFF
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    -DCMAKE_PROJECT_pic_project_INCLUDE=${CMAKE_SOURCE_DIR}/cmake/add-extra-properties.cmake
    -DEXTRA_INCLUDE_DIRECTORIES=$<TARGET_PROPERTY:${COMPONENT_TARGET},INTERFACE_INCLUDE_DIRECTORIES>
  CMAKE_CACHE_ARGS
    -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON
    -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
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
