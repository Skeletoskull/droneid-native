add_library(droneid_locate SHARED
    locate_api.c
    locate_file.c
    ofdm_demod.c
    zc_sync.c
    qpsk_decode.c
    turbo_deratematch.c
    turbo_decode.c
    duml_parser.c
    crc16.c
    json_serializer.c)

target_include_directories(droneid_locate
    PUBLIC  ${PROJECT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
            ${PROJECT_SOURCE_DIR}/src/dsp)

target_compile_definitions(droneid_locate PRIVATE DRONEID_LOCATE_EXPORTS)

# Statically link DSP primitives — no external DSP dependency at runtime
target_link_libraries(droneid_locate PRIVATE dsp_primitives)

# Hide all symbols except those explicitly marked LOCATE_API
set_target_properties(droneid_locate PROPERTIES
    C_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON)

# On Windows, also produce an import library (.lib) alongside the .dll
if(WIN32)
    set_target_properties(droneid_locate PROPERTIES
        WINDOWS_EXPORT_ALL_SYMBOLS OFF)
endif()
