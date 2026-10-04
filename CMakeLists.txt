cmake_minimum_required(VERSION 3.16)
project(droneid_native
    VERSION 1.0.0
    DESCRIPTION "Native real-time DJI DroneID detector and decoder"
    LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Default to Release for single-config generators
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ── Options ───────────────────────────────────────────────────────────────────
option(USE_FFTW3            "Use FFTW3 for FFT (falls back to KissFFT if not found)" OFF)
option(DRONEID_BUILD_TESTS  "Build the unit tests"                                   ON)
option(DRONEID_BUILD_TOOLS  "Build the command-line tools in tools/"                 ON)
set(DRONEID_TEST_CAPTURE "" CACHE FILEPATH
    "Optional IQ capture (interleaved float32) for the end-to-end validation test")
set(DRONEID_TEST_CAPTURE_RATE "50000000" CACHE STRING
    "Sample rate of DRONEID_TEST_CAPTURE in Hz")

# ── FFT backend selection ─────────────────────────────────────────────────────
set(FFT_BACKEND "kissfft")
if(USE_FFTW3)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(FFTW3F QUIET fftw3f)
    endif()
    if(NOT FFTW3F_FOUND)
        find_library(FFTW3F_LIBRARIES NAMES fftw3f libfftw3f-3)
        find_path(FFTW3F_INCLUDE_DIRS NAMES fftw3.h)
        if(FFTW3F_LIBRARIES AND FFTW3F_INCLUDE_DIRS)
            set(FFTW3F_FOUND TRUE)
        endif()
    endif()
    if(FFTW3F_FOUND)
        set(FFT_BACKEND "fftw3")
    else()
        message(STATUS "FFTW3 not found - falling back to KissFFT")
    endif()
endif()

# ── Compiler settings ─────────────────────────────────────────────────────────
if(MSVC)
    add_compile_options(/W3 /wd4996)
else()
    add_compile_options(-Wall -Wextra -Wno-unused-parameter -fvisibility=hidden)
endif()

# Install-dir variables must be defined before the sub-directories use them.
include(GNUInstallDirs)

# ── Libraries ─────────────────────────────────────────────────────────────────
add_subdirectory(src/dsp)
add_subdirectory(src/detect)
add_subdirectory(src/locate)
add_subdirectory(src/bridge)

# Windows: collect every runtime DLL and executable in one folder so tests,
# tools and the bridge resolve each other without PATH changes.
if(WIN32)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
    set_target_properties(droneid_detect droneid_locate droneid_bridge PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
endif()

# ── Tests and tools ───────────────────────────────────────────────────────────
if(DRONEID_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(DRONEID_BUILD_TOOLS)
    add_subdirectory(tools)
endif()

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS droneid_detect droneid_locate droneid_bridge
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(FILES
    include/droneid_detect.h
    include/droneid_locate.h
    include/droneid_bridge.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(FILES examples/c_example.c
    DESTINATION ${CMAKE_INSTALL_DATADIR}/droneid_native/examples)

# ── Summary ───────────────────────────────────────────────────────────────────
message(STATUS "droneid_native ${PROJECT_VERSION}")
message(STATUS "  FFT backend : ${FFT_BACKEND}")
message(STATUS "  Tests       : ${DRONEID_BUILD_TESTS}")
message(STATUS "  Tools       : ${DRONEID_BUILD_TOOLS}")
