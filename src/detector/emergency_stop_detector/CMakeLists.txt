message("cmake emergency stop detector...")

include_directories(
    include/emergency_stop_detector
    src
)

add_library(emergency_stop_detector SHARED
    src/emergency_stop_detector.cc
)

target_link_libraries(emergency_stop_detector
    pthread
    data_center
    event_center
    geometry
    hsm
    log
    timer
    config
)

if (NOT ALL)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()

if ((TEST) AND (NOT ALL))
    message("cmake test emergency_stop_detector...")
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)

    add_executable(emergency_stop_detector
        test/emergency_stop_detector.cc
    )
    target_link_libraries(emergency_stop_detector
        emergency_stop_detector
    )
endif()
