



include_directories(${PROJECT_SOURCE_DIR}/3rdparty/g2o/include/EXTERNAL/csparse)

# if (BUILD_PLATFORM STREQUAL "x86_64")
# message("build teb_planner test(x86_64)...")
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/x86_64/release)
# elseif (BUILD_PLATFORM STREQUAL "arm")
# message("build teb_planner test(arm)...")
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/arm/release)
# endif()

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)

add_executable(test_optim_node
    test_optim_node.cpp
)

target_link_libraries(test_optim_node
    teb_planner  
    g2o_ext_csparse
    g2o_core
    g2o_csparse_extension
    g2o_stuff
    timer
    config
    opencv_core
    boost_system
    boost_thread
    pthread
)

