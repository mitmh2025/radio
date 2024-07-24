cmake_minimum_required( VERSION 3.12 )

idf_component_register(REQUIRES mbedtls)

foreach(LIB IN ITEMS mbedtls newlib lwip vfs esp_system freertos xtensa esp_hw_support soc esp_common heap esp_rom)
  message(STATUS "Getting include directories for ${LIB}")
  idf_component_get_property(tmp_lib ${LIB} COMPONENT_LIB)
  get_target_property(tmp_includes ${tmp_lib} INTERFACE_INCLUDE_DIRECTORIES)
  list(APPEND INCLUDES ${tmp_includes})
endforeach()

list(TRANSFORM INCLUDES PREPEND "-I")
list(JOIN INCLUDES " " SRTP_CFLAGS)
string(APPEND SRTP_CFLAGS " -D_GNU_SOURCE -Wno-system-headers")

set(SRTP_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/srtp_install)

include(ExternalProject)

ExternalProject_Add(srtp
  GIT_REPOSITORY https://github.com/cisco/libsrtp
  GIT_TAG 5f8dfbf7eedbc2fa737cf0a2931b464f10396379
  DOWNLOAD_EXTRACT_TIMESTAMP true
  PATCH_COMMAND sed -i "" -e "s/CMAKE_C_STANDARD 99/CMAKE_C_STANDARD 11/" CMakeLists.txt &&
                sed -i "" -e "s/srtp_remove_stream(srtp_t session, unsigned int ssrc/srtp_remove_stream(srtp_t session, uint32_t ssrc/" include/srtp.h
  CMAKE_ARGS
    -DENABLE_MBEDTLS=ON
    -DLIBSRTP_TEST_APPS=OFF
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    -DBUILD_SHARED_LIBS=OFF
    -DENABLE_WARNINGS_AS_ERRORS=OFF
    -DMBEDTLS_INCLUDE_DIRS=$<TARGET_PROPERTY:mbedtls,INTERFACE_INCLUDE_DIRECTORIES>
    -DMBEDTLS_LIBRARY=$<TARGET_FILE:mbedtls>
    -DMBEDX509_LIBRARY=$<TARGET_FILE:mbedx509>
    -DMBEDCRYPTO_LIBRARY=$<TARGET_FILE:mbedcrypto>
    -DCMAKE_C_FLAGS=${SRTP_CFLAGS}
    -DCMAKE_CXX_FLAGS=${SRTP_CFLAGS}
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
  INSTALL_DIR ${SRTP_INSTALL_DIR}
  BUILD_BYPRODUCTS ${SRTP_INSTALL_DIR}/lib/libsrtp3.a
)

add_prebuilt_library(srtp_lib "${SRTP_INSTALL_DIR}/lib/libsrtp3.a")
target_include_directories(srtp_lib INTERFACE "${SRTP_INSTALL_DIR}/include")
add_dependencies(srtp_lib srtp)
target_link_libraries(${COMPONENT_LIB} INTERFACE srtp_lib)
