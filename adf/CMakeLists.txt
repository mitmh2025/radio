cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

list(APPEND EXTRA_COMPONENT_DIRS "esp-adf/components")

project(radio)

if(EXISTS ${SDKCONFIG} AND sdkconfig.defaults IS_NEWER_THAN ${SDKCONFIG})
  message(FATAL_ERROR "sdkconfig.defaults is newer than ${SDKCONFIG}. Because ESP-IDF doesn't pull new defaults after sdkconfig has been populated, the new settings won't take effect. Please remove ${SDKCONFIG} so that it can be regenerated, and try again")
endif()
