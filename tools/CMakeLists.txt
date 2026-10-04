# Command-line tools: each one is a thin wrapper over the public API.
set(DRONEID_TOOLS
    droneid_scan        # locate_droneid_file: detect + decode a capture
    droneid_candidates  # detect_droneid_file: list burst candidates
    droneid_bridge_cli  # droneid_process: the LabVIEW call path
    droneid_validate    # end-to-end checks on a capture (used by ctest)
    turbo_selftest)     # turbo FEC engine self-test

foreach(tool ${DRONEID_TOOLS})
    add_executable(${tool} ${tool}.c)
    target_link_libraries(${tool} PRIVATE droneid_detect droneid_locate droneid_bridge)
    if(NOT WIN32)
        target_link_libraries(${tool} PRIVATE m)
    endif()
endforeach()

# The documented example is built too, so it can never drift from the API.
add_executable(c_example ${PROJECT_SOURCE_DIR}/examples/c_example.c)
target_link_libraries(c_example PRIVATE droneid_detect droneid_locate)

if(DRONEID_BUILD_TESTS)
    add_test(NAME turbo_selftest COMMAND turbo_selftest)
    set_tests_properties(turbo_selftest PROPERTIES LABELS "unit" TIMEOUT 120)
endif()

install(TARGETS ${DRONEID_TOOLS} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
