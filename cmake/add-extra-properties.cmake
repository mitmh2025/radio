option(EXTRA_INCLUDE_DIRECTORIES "Extra include directories" "")
if(EXTRA_INCLUDE_DIRECTORIES)
  include_directories(${EXTRA_INCLUDE_DIRECTORIES})
endif()

option(EXTRA_COMPILE_DEFINITIONS "Extra definitions" "")
if(EXTRA_COMPILE_DEFINITIONS)
  add_compile_definitions(${EXTRA_COMPILE_DEFINITIONS})
endif()