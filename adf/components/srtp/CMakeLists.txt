cmake_minimum_required(VERSION 3.12)

idf_component_register(PRIV_REQUIRES mbedtls)

set(SRTP_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/srtp_install)
idf_component_set_property(${COMPONENT_NAME} SRTP_PREFIX ${SRTP_INSTALL_DIR})

include(ExternalProject)

ExternalProject_Add(srtp
  GIT_REPOSITORY https://github.com/cisco/libsrtp
  GIT_TAG fd08747fa6800b321d53e15feb34da12dc697dee
  PATCH_COMMAND
    sed -i "" -e "s/CMAKE_C_STANDARD 99/CMAKE_C_STANDARD 11/" CMakeLists.txt &&
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
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
  CMAKE_CACHE_ARGS
    -DHAVE_NETINET_IN_H:BOOL=ON
  INSTALL_DIR ${SRTP_INSTALL_DIR}
  BUILD_BYPRODUCTS ${SRTP_INSTALL_DIR}/lib/libsrtp2.a
)

add_prebuilt_library(srtp_lib "${SRTP_INSTALL_DIR}/lib/libsrtp2.a")
target_include_directories(srtp_lib INTERFACE "${SRTP_INSTALL_DIR}/include")
add_dependencies(srtp_lib srtp)
target_link_libraries(${COMPONENT_LIB} INTERFACE srtp_lib)
