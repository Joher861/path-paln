cmake_minimum_required(VERSION 3.5)

project(planning)

set (LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

if (BUILD_PLATFORM STREQUAL "x86_64")
    message("build x86_64...")
    set (CMAKE_FIND_ROOT_PATH /)
    set (LIBRARY_OUTPUT_PATH ${LIBRARY_OUTPUT_PATH}/x86_64)
    set (CMAKE_CXX_COMPILER "/usr/bin/g++-7")
    set (CMAKE_C_COMPILER "/usr/bin/gcc-7")
elseif (BUILD_PLATFORM STREQUAL "arm")
    message("build arm...")
    set (COMPILER_DIR /opt/rockchip-cc-tools/host/usr)
    set (CMAKE_FIND_ROOT_PATH ${COMPILER_DIR}/aarch64-buildroot-linux-gnu/sysroot)
    set (LIBRARY_OUTPUT_PATH ${LIBRARY_OUTPUT_PATH}/arm)
    set (CMAKE_CXX_COMPILER ${COMPILER_DIR}/bin/aarch64-linux-g++)
    set (CMAKE_C_COMPILER ${COMPILER_DIR}/bin/aarch64-linux-gcc)
elseif (BUILD_PLATFORM STREQUAL "ros")
    message("build ros...")
    set (CMAKE_FIND_ROOT_PATH /)
    set (LIBRARY_OUTPUT_PATH ${LIBRARY_OUTPUT_PATH}/ros)
    set (CMAKE_CXX_COMPILER "/usr/bin/g++-7")
    set (CMAKE_C_COMPILER "/usr/bin/gcc-7")
else()
endif()

if (BUILD_TYPE STREQUAL "release")
    message("build release...")
    set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    # set (CMAKE_CXX_FLAGS
    #     "-o3 -Wall -Wextra -Wno-unused-parameter -Wno-unused-variable -std=c++17 -rdynamic")  
    set (LIBRARY_OUTPUT_PATH ${LIBRARY_OUTPUT_PATH}/release)
    message("planning lib out path ${LIBRARY_OUTPUT_PATH}")
elseif (BUILD_TYPE STREQUAL "debug")
    message("build debug...")
    set (CMAKE_CXX_FLAGS
        "-g -Wall -Wextra -Wno-unused-parameter -Wno-unused-variable -std=c++17 -rdynamic ${RPATH_FLAG}")
    set (LIBRARY_OUTPUT_PATH ${LIBRARY_OUTPUT_PATH}/debug)
else()
endif()

include_directories(
    include
    include/common
    include/utils
    include/data
    include/controller
    include/map
    include/planner
    # include/planner/global_teb_planner
    include/detector
    3rdparty
    3rdparty/g2o/include
    3rdparty/g2o/include/g2o/EXTERNAL/csparse
    3rdparty/linux_ipc_wrappers/
    3rdparty/opencv/include
    3rdparty/opencv/include/opencv
    3rdparty/opencv/include/opencv2
#    ${CMAKE_FIND_ROOT_PATH}/usr/include
)

link_directories (
    ${LIBRARY_OUTPUT_PATH}

    # lib
    3rdparty/opencv/lib
#    ${CMAKE_FIND_ROOT_PATH}/usr/lib
)

if (BUILD_EXAMPLE)
    message("build test")
    set(TEST 1)
endif()

if (BUILD_MODULE STREQUAL "all")
    message("build module: all")
    set(ALL 1)
endif()

if (BUILD_MODULE STREQUAL "data")
    message("build module: data")
    set(DATA 1)
endif()

if (BUILD_MODULE STREQUAL "data_center")
    message("build module: data_center")
    set(DATA_CENTER 1)
endif()

if (BUILD_MODULE STREQUAL "event_center")
    message("build module: event_center")
    set(EVENT_CENTER 1)
endif()

if (BUILD_MODULE STREQUAL "utils")
    message("build module: utils")
    set(UTILS 1)
endif()

if (BUILD_MODULE STREQUAL "config")
    message("build module: config")
    set(CONFIG 1)
endif()

if (BUILD_MODULE STREQUAL "geometry")
    message("build module: geometry")
    set(GEOMETRY 1)
endif()

if (BUILD_MODULE STREQUAL "hsm")
    message("build module: hsm")
    set(HSM 1)
endif()

if (BUILD_MODULE STREQUAL "log")
    message("build module: log")
    set(LOG 1)
endif()

if (BUILD_MODULE STREQUAL "timer")
    message("build module: timer")
    set(TIMER 1)
endif()

if (BUILD_MODULE STREQUAL "config")
    message("build module: config")
    set(CONFIG 1)
endif()

if (BUILD_MODULE STREQUAL "map")
    message("build module: map")
    set(MAP 1)
endif()

if (BUILD_MODULE STREQUAL "clean_map")
    message("build module: clean_map")
    set(CLEAN_MAP 1)
endif()

if (BUILD_MODULE STREQUAL "local_map")
    message("build module: local_map")
    set(LOCAL_MAP 1)
endif()

if (BUILD_MODULE STREQUAL "detector")
    message("build module: detector")
    set(DETECTOR 1)
endif()

if (BUILD_MODULE STREQUAL "emergency_stop_detector")
    message("build module: emergency_stop_detector")
    set(EMERGENCY_STOP_DETECTOR 1)
endif()

if (BUILD_MODULE STREQUAL "obstacle_detector")
    message("build module: obstacle_detector")
    set(OBSTACLE_DETECTOR 1)
endif()

if (BUILD_MODULE STREQUAL "collision_detector")
    message("build module: collision_detector")
    set(COLLISION_DETECTOR 1)
endif()

if (BUILD_MODULE STREQUAL "controller")
    message("build module: controller")
    set(CONTROLLER 1)
endif()

if (BUILD_MODULE STREQUAL "task")
    message("build module: task")
    set(TASK 1)
endif()

if (BUILD_MODULE STREQUAL "speed_controller")
    message("build module: speed_controller")
    set(SPEED_CONTROLLER 1)
endif()

if (BUILD_MODULE STREQUAL "rotate_planner")
    message("build module: rotate_planner")
    set(ROTATE_PLANNER 1)
endif()

if (BUILD_MODULE STREQUAL "basic_planner")
message("build module: basic_planner")
set(BASIC_PLANNER 1)
endif()

if (BUILD_MODULE STREQUAL "mpc_planner")
    message("build module: mpc_planner")
    set(MPC_PLANNER 1)
endif()

if (BUILD_MODULE STREQUAL "pure_pursuit_planner")
    message("build module: pure_pursuit_planner")
    set(PURE_PURSUIT_PLANNER 1)
endif()

# if (BUILD_MODULE STREQUAL "teb_planner")
#     message("build module: teb_planner")
#     set(TEB_PLANNER 1)
# endif()

if (BUILD_MODULE STREQUAL "wall_follow_planner")
    message("build module: wall_follow_planner")
    set(WALL_FOLLOW_PLANNER 1)
endif()

if (BUILD_MODULE STREQUAL "navigation_planner")
    message("build module: navigation_planner")
    set(NAVIGATION_PLANNER 1)
endif()

if (BUILD_MODULE STREQUAL "path_follow_planner")
    message("build module: path_follow_planner")
    set(PATH_FOLLOW_PLANNER 1)
endif()

if (BUILD_MODULE STREQUAL "ccpp_coverage_planner")
    message("build module: ccpp_coverage_planner")
    set(CCPP_COVERAGE_PLANNER 1)
endif()

if (BUILD_MODULE STREQUAL "global_teb_planner")
    message("build module: global_teb_planner")
    set(GLOBAL_TEB_PLANNER 1)
endif()

if (BUILD_MODULE STREQUAL "edge_follow_planner")
    message("build module: edge_follow_planner")
    set(GLOBAL_TEB_PLANNER 1)
endif()

message("cmaking...")
if (TEST)
    message("cmaking test...")
endif()

if (UTILS OR CONFIG OR GEOMETRY OR HSM OR LOG OR TIMER OR ALL)
    add_subdirectory(src/utils)
endif()

if (DATA OR DATA_CENTER OR EVENT_CENTER OR ALL)
    add_subdirectory(src/data)
endif()

if (MAP OR CLEAN_MAP OR LOCAL_MAP OR ALL)
    add_subdirectory(src/map)
endif()

if (DETECTOR OR EMERGENCY_STOP_DETECTOR OR OBSTACLE_DETECTOR OR COLLISION_DETECTOR OR ALL)
    add_subdirectory(src/detector)
endif()

if (CONTROLLER OR SPEED_CONTROLLER OR TASK
    OR ROTATE_PLANNER OR BASIC_PLANNER OR MPC_PLANNER OR PURE_PURSUIT_PLANNER
    OR TEB_PLANNER OR WALL_FOLLOW_PLANNER OR FDFS_PLANNER OR ALL)
    add_subdirectory(src/controller)
endif()

if (PLANNER OR NAVIGATION_PLANNER OR PATH_FOLLOW_PLANNER OR CCPP_COVERAGE_PLANNER OR GLOBAL_TEB_PLANNER OR EDGE_FOLLOW_PLANNER OR ALL)
    add_subdirectory(src/planner)
endif()

if (ALL)
    message("cmake all...")
    # set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
    # set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
endif()
