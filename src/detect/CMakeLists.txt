add_library(droneid_detect SHARED
    detect_api.c
    packet_detector.c
    stft.c
    cfo_estimator.c
    resampler.c)

target_include_directories(droneid_detect
    PUBLIC  ${PROJECT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
            ${PROJECT_SOURCE_DIR}/src/dsp)

target_compile_definitions(droneid_detect PRIVATE DRONEID_DETECT_EXPORTS)

# Statically link DSP primitives — no external DSP dependency at runtime
target_link_libraries(droneid_detect PRIVATE dsp_primitives)

# Hide all symbols except DETECT_API exports
set_target_properties(droneid_detect PROPERTIES
    C_VISIBILITY_PRESET   hidden
    VISIBILITY_INLINES_HIDDEN ON)

# On Windows, also produce the .lib import library
if(WIN32)
    set_target_properties(droneid_detect PROPERTIES
        WINDOWS_EXPORT_ALL_SYMBOLS OFF)
endif()
