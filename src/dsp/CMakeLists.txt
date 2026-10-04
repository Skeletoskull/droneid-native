set(DSP_SOURCES
    fft.c
    zc_sequence.c
    gold_sequence.c
    fshift.c
    resample.c
    correlation.c
    welch_psd.c
    stft.c)

add_library(dsp_primitives STATIC ${DSP_SOURCES})

target_include_directories(dsp_primitives
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE ${PROJECT_SOURCE_DIR}/include)

# FFT backend
if(FFT_BACKEND STREQUAL "fftw3")
    target_compile_definitions(dsp_primitives PUBLIC USE_FFTW3)
    target_include_directories(dsp_primitives PRIVATE ${FFTW3F_INCLUDE_DIRS})
    target_link_libraries(dsp_primitives PUBLIC ${FFTW3F_LIBRARIES})
else()
    target_sources(dsp_primitives PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/kissfft/kiss_fft.c)
    target_include_directories(dsp_primitives PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/kissfft)
endif()

# Thread safety
find_package(Threads REQUIRED)
target_link_libraries(dsp_primitives PUBLIC Threads::Threads)

# OpenMP — parallelises the STFT detection envelope (the dominant cost at high
# sample rates). Optional: falls back to a serial loop if not found.
find_package(OpenMP QUIET COMPONENTS C)
if(OpenMP_C_FOUND)
    message(STATUS "OpenMP: enabled (${OpenMP_C_FLAGS})")
    target_link_libraries(dsp_primitives PUBLIC OpenMP::OpenMP_C)
else()
    message(STATUS "OpenMP: not found — STFT detection runs single-threaded")
endif()

# Math library (Linux)
if(UNIX)
    target_link_libraries(dsp_primitives PUBLIC m)
endif()

# Position-independent code (needed when linking into shared libs)
set_target_properties(dsp_primitives PROPERTIES POSITION_INDEPENDENT_CODE ON)
