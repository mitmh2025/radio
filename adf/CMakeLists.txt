cmake_minimum_required(VERSION 3.5)

set(CMAKE_BUILD_TYPE Debug)

include($ENV{ADF_PATH}/CMakeLists.txt)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(radio)

if(EXISTS ${SDKCONFIG} AND ${CMAKE_SOURCE_DIR}/sdkconfig.defaults IS_NEWER_THAN ${SDKCONFIG})
  message(FATAL_ERROR "sdkconfig.defaults is newer than ${SDKCONFIG}. Because ESP-IDF doesn't pull new defaults after sdkconfig has been populated, the new settings won't take effect. Please remove ${SDKCONFIG} so that it can be regenerated, and try again")
endif()
