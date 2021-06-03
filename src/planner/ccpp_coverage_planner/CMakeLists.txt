
# cmake 最低版本需求

# cmake_minimum_required(VERSION 2.8.3)
# 工程名称
#project(IPA_CCPP)

# 设置
# set(CMAKE_CXX_STANDARD 11)
# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

message("cmake ccpp_coverage_planner planner...")
add_compile_options(-fPIC)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")


set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include_directories(
    ../path_follow_planner/include/path_follow_planner
    ../path_follow_planner/include/op_local_planner
    ../navigation_planner
    include/ccpp_coverage_planner
    src
)


link_directories (
    ${PROJECT_SOURCE_DIR}/3rdparty/linux_ipc_wrappers/lib/${BUILD_PLATFORM}
)

find_package(Boost REQUIRED COMPONENTS system chrono thread)

# find_package(OpenCV REQUIRED)
set(OpenCV_LIBS
    opencv_core
    opencv_video
    opencv_imgproc
    opencv_calib3d
    opencv_videoio
    opencv_highgui
    opencv_imgcodecs)

include_directories( 
    include
	# ${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_library(ccpp_coverage_planner SHARED src/ccpp_coverage_planner.cpp
                                        src/ccpp_coverage_planner_config.h 
                                        src/ccpp_coverage_planner_log.h
                                        src/pre_process.cpp
                                        src/map_accessibility_analysis.cpp
                                        src/map_accessibility_analysis.h
                                        src/room_rotator.cpp
                                        src/room_rotator.h
                                        src/A_star_pathplanner.cpp
                                        src/A_star_pathplanner.h
                                        src/node.cpp
                                        src/node.h
                                        src/nearest_neighbor_TSP.cpp
                                        src/nearest_neighbor_TSP.h
                                        src/meanshift2d.cpp
                                        src/meanshift2d.h
                                        src/genetic_TSP.cpp
                                        src/genetic_TSP.h
                                        src/grid.h
                                        src/histogram.h
                                        src/fov_to_robot_mapper.cpp
                                        src/boustrophedon_explorator.cpp
                                        src/boustrophedon_explorator.h)

# 编译源码生成目标
target_link_libraries(ccpp_coverage_planner
    path_follow_planner
    navigation_planner
    hybrid_A_star_lib
    geometry
    data_center
    event_center
    hsm
    log
    timer
    config
    task
    clean_map
    ${OpenCV_LIBS}
    # ${Boost_LIBRARIES}
    boost_system
    boost_chrono
    boost_thread
    
    # Threads::Threads

    )

add_executable(test_ccpp_coverage_planner test/test_ccpp_coverage_planner.cpp)
target_link_libraries(test_ccpp_coverage_planner
ccpp_coverage_planner
)

if (NOT ALL)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()
