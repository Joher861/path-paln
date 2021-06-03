message("cmake rotate planner...")

include_directories(
    include/rotate_planner
    src
)

add_library(rotate_planner SHARED
    src/rotate_planner.cc
)

target_link_libraries(rotate_planner
    pthread
    data_center
    event_center
    geometry
    hsm
    log
    timer
    config
    task
    speed_controller
)

if (NOT ALL)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()

if ((TEST) AND (NOT ALL))
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)

    add_executable(test_rotate_planner
        test/test_rotate_planner.cc
    )
    target_link_libraries(test_rotate_planner
        rotate_planner
    )
endif()
