add_library(droneid_bridge SHARED
    droneid_bridge.c)

target_include_directories(droneid_bridge
    PUBLIC  ${PROJECT_SOURCE_DIR}/include
    PRIVATE ${PROJECT_SOURCE_DIR}/src/dsp)

target_compile_definitions(droneid_bridge PRIVATE DRONEID_BRIDGE_EXPORTS)

# The bridge calls the public detect and locate APIs
target_link_libraries(droneid_bridge PRIVATE
    droneid_detect
    droneid_locate)
