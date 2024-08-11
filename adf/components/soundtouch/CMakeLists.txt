idf_component_register(
  SRCS
  soundtouch_filter.cpp
  soundtouch/source/SoundTouch/AAFilter.cpp
  soundtouch/source/SoundTouch/BPMDetect.cpp
  soundtouch/source/SoundTouch/cpu_detect_x86.cpp
  soundtouch/source/SoundTouch/FIFOSampleBuffer.cpp
  soundtouch/source/SoundTouch/FIRFilter.cpp
  soundtouch/source/SoundTouch/InterpolateCubic.cpp
  soundtouch/source/SoundTouch/InterpolateLinear.cpp
  soundtouch/source/SoundTouch/InterpolateShannon.cpp
  soundtouch/source/SoundTouch/mmx_optimized.cpp
  soundtouch/source/SoundTouch/PeakFinder.cpp
  soundtouch/source/SoundTouch/RateTransposer.cpp
  soundtouch/source/SoundTouch/SoundTouch.cpp
  soundtouch/source/SoundTouch/sse_optimized.cpp
  soundtouch/source/SoundTouch/TDStretch.cpp
  INCLUDE_DIRS
  "."
  soundtouch/include
  REQUIRES
  audio_pipeline)
target_compile_definitions(${COMPONENT_LIB} PRIVATE SOUNDTOUCH_INTEGER_SAMPLES)
target_compile_definitions(${COMPONENT_LIB} PUBLIC ST_NO_EXCEPTION_HANDLING SOUNDTOUCH_PREVENT_CLICK_AT_RATE_CROSSOVER)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unknown-pragmas)
