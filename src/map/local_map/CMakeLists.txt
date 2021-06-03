message("cmake local map...")

include_directories(
    include/local_map
    src
    ${PROJECT_SOURCE_DIR}/3rdparty/linux_ipc_wrappers
)

link_directories (
    ${PROJECT_SOURCE_DIR}/3rdparty/linux_ipc_wrappers/lib/${BUILD_PLATFORM}
)

add_library(local_map SHARED
    src/local_map.cc
    src/local_map_viewer.cc
    src/local_base_layer.cc
    src/local_uint8_layer.cc
    src/local_window_map.cc
    src/sdf_map.cc
    src/array_parser.cc
    src/costmap_math.cc
    src/footprint.cc
    ../src/grid_map.cc
    ../src/inflate_layer.cc
)

target_link_libraries(local_map
    log
    config
    geometry
    timer
    opencv_core
    opencv_imgcodecs
    opencv_imgproc
    linux_ipc_wrappers
    
)

if (NOT ALL)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()

if ((TEST) AND (NOT ALL))
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)

    add_executable(test_local_map
        test/test_local_map.cc
    )
    target_link_libraries(test_local_map
        local_map
        data_center
        event_center
        pthread
        geometry
    )
endif()
