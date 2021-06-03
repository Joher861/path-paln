message("cmake navigation_planner planner...")

include_directories(
    src
    ../path_follow_planner/include/path_follow_planner
    ../path_follow_planner/include/op_local_planner
    # ../../controller/local_planner/mpc_planner/include/mpc_planner
    # src/controller/local_planner/mpc_planner/include/mpc_planner
    include/navigation_planner
)



link_directories (
    ${PROJECT_SOURCE_DIR}/3rdparty/linux_ipc_wrappers/lib/${BUILD_PLATFORM}
)

set(OpenCV_LIBS
    opencv_core
    opencv_video
    opencv_imgproc
    opencv_calib3d
    opencv_videoio
    opencv_highgui
    opencv_imgcodecs)


add_library(hybrid_A_star_lib  SHARED
               src/hybrid_breadth_first.cpp
           )

add_library(navigation_planner SHARED
    src/navigation_planner.cpp
)

target_link_libraries(navigation_planner
    pthread
    data_center
    event_center
    geometry
    hsm
    log
    timer
    config
    task
    hybrid_A_star_lib
    path_follow_planner
    clean_map
    # speed_controller
    # navigation_planner
    ${OpenCV_LIBS}
)

    add_executable(test_navigation_planner
        test/test_navigation_planner.cpp
    )
    target_link_libraries(test_navigation_planner
    navigation_planner
    )

if (NOT ALL)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()

# if ((TEST) AND (NOT ALL))
#     set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)

#     add_executable(test_coverage_planner
#         test/test_coverage_planner.cc
#     )
#     target_link_libraries(test_coverage_planner
#         coverage_planner
#         basic_planner
#         rotate_planner
#         follower_planner
#         edge_planner
#         pass_planner
#     )
# endif()
