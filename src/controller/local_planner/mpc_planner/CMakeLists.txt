message("cmake mpc planner...")

include_directories(
    include/mpc_planner
    src
)

link_directories (
    # lib
)
add_library(mpc_planner SHARED
    src/mpc_planner.cc
    src/robot_nlp.cpp
)

target_link_libraries(mpc_planner
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
    ipopt
)


if (NOT ALL)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()

if ((TEST) AND (NOT ALL))
    message("cmake test mpc_controller...")
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)

    add_executable(test_mpc_planner
        test/test_mpc_planner.cc
    )
    target_link_libraries(test_mpc_planner
        mpc_planner
    )
endif()
