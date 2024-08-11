idf_component_register(
  SRCS
  stretch_filter.cpp
  INCLUDE_DIRS
  "."
  signalsmith-stretch
  REQUIRES
  audio_pipeline)

target_compile_options(${COMPONENT_LIB} PRIVATE -O3)
