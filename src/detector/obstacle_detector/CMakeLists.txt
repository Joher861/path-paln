message("cmake obstacle detector...")

include_directories(
    include/obstacle_detector
    src
)

add_library(obstacle_detector SHARED
    src/obstacle_detector.cc
)

target_link_libraries(obstacle_detector
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
    message("cmake test obstacle_detector...")
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)

    add_executable(obstacle_detector
        test/obstacle_detector.cc
    )
    target_link_libraries(obstacle_detector
        obstacle_detector
    )
endif()
