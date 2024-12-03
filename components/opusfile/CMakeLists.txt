FILE(GLOB OPUSFILE_SOURCES "opusfile/src/*.c")
idf_component_register(
    SRCS
        opusfile/src/info.c
        opusfile/src/internal.c
        opusfile/src/opusfile.c
        opusfile/src/stream.c
    INCLUDE_DIRS
        "opusfile/include"
    REQUIRES
        opus
        ogg
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE -DOP_DISABLE_FLOAT_API -DOP_FIXED_POINT)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=maybe-uninitialized)
