idf_component_register(
  SRCS
  "rubberband_filter.cpp"
  "rubberband/src/rubberband-c.cpp"
  "rubberband/src/RubberBandStretcher.cpp"
  "rubberband/src/RubberBandLiveShifter.cpp"
  "rubberband/src/faster/AudioCurveCalculator.cpp"
  "rubberband/src/faster/CompoundAudioCurve.cpp"
  "rubberband/src/faster/HighFrequencyAudioCurve.cpp"
  "rubberband/src/faster/SilentAudioCurve.cpp"
  "rubberband/src/faster/PercussiveAudioCurve.cpp"
  "rubberband/src/faster/R2Stretcher.cpp"
  "rubberband/src/faster/StretcherChannelData.cpp"
  "rubberband/src/faster/StretcherProcess.cpp"
  "rubberband/src/common/Allocators.cpp"
  "rubberband/src/common/FFT.cpp"
  "rubberband/src/common/Log.cpp"
  "rubberband/src/common/Profiler.cpp"
  "rubberband/src/common/Resampler.cpp"
  "rubberband/src/common/StretchCalculator.cpp"
  "rubberband/src/common/sysutils.cpp"
  "rubberband/src/common/mathmisc.cpp"
  "rubberband/src/common/Thread.cpp"
  "rubberband/src/finer/R3Stretcher.cpp"
  "rubberband/src/finer/R3LiveShifter.cpp"
  "rubberband/src/common/BQResampler.cpp"
  INCLUDE_DIRS
  "."
  "rubberband/rubberband"
  REQUIRES audio_pipeline)
add_compile_definitions(-DUSE_BUILTIN_FFT -DUSE_BQRESAMPLER -DUSE_PTHREADS -DLACK_POSIX_MEMALIGN -DUSE_OWN_ALIGNED_MALLOC -DNO_EXCEPTIONS -DPROCESS_SAMPLE_TYPE=float)
target_link_libraries(${COMPONENT_LIB} PUBLIC m)
