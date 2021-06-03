message("cmake clean map...")

include_directories(
    src
    include/clean_map
)

add_library(clean_map SHARED
    src/clean_map.cc
    ../src/grid_map.cc
    ../src/inflate_layer.cc
    ./src/local_layer.cpp
)

target_link_libraries(clean_map
    data_center
    event_center    
    log
    opencv_core
    opencv_imgcodecs
    opencv_imgproc
    config
    geometry
    timer
    task
    pthread
)

if (NOT ALL)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()

if ((TEST) AND (NOT ALL))
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)

    add_executable(test_clean_map
        test/test_clean_map.cc
    )
    target_link_libraries(test_clean_map
        clean_map
        pthread
        opencv_core
        opencv_imgcodecs
        opencv_imgproc
        geometry
    )
endif()
