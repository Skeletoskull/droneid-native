# ── Unit tests (synthetic signals, no capture required) ───────────────────────
foreach(test_name test_dsp test_detect test_locate test_bridge)
    add_executable(${test_name} ${test_name}.c)
    target_include_directories(${test_name} PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src/dsp)
    target_link_libraries(${test_name} PRIVATE
        droneid_detect
        droneid_locate
        droneid_bridge
        dsp_primitives)
    add_test(NAME ${test_name} COMMAND ${test_name})
    set_tests_properties(${test_name} PROPERTIES LABELS "unit" TIMEOUT 300)
endforeach()

# ── Optional end-to-end test against a real capture ───────────────────────────
#   cmake -B build -DDRONEID_TEST_CAPTURE=path/to/capture.bin
#                  -DDRONEID_TEST_CAPTURE_RATE=50000000
if(DRONEID_TEST_CAPTURE)
    if(NOT EXISTS "${DRONEID_TEST_CAPTURE}")
        message(FATAL_ERROR "DRONEID_TEST_CAPTURE not found: ${DRONEID_TEST_CAPTURE}")
    endif()
    if(NOT DRONEID_BUILD_TOOLS)
        message(FATAL_ERROR "DRONEID_TEST_CAPTURE requires DRONEID_BUILD_TOOLS=ON")
    endif()
    add_test(NAME validate_capture
        COMMAND droneid_validate "${DRONEID_TEST_CAPTURE}" ${DRONEID_TEST_CAPTURE_RATE})
    set_tests_properties(validate_capture PROPERTIES
        LABELS "integration" TIMEOUT 600)
endif()
