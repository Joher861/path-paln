message("cmake path_follow_planner planner...")

include_directories(
    include/path_follow_planner
    ../../../include/planner/global_teb_planner
    src
)


add_library(path_follow_planner SHARED
    src/path_follow_planner.cpp
)

target_link_libraries(path_follow_planner
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
    pure_pursuit_planner
    mpc_planner
    collision_detector
    global_teb_planner
)

if (NOT ALL)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()

# if ((TEST) AND (NOT ALL))
#     set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)
#     add_executable(test_path_follow_planner
#         test/test_path_follow_planner.cc
#     )
#     target_link_libraries(path_follow_planner
#     )
# endif()
