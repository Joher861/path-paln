message("cmake log...")

include_directories(
    include/log
    include/log/include
)

add_library(log SHARED
    src/log_manager.cc
    src/log_func.cc
)

target_link_libraries(log
    pthread
    timer
    config
    opencv_core
    opencv_imgcodecs
)

if (NOT ALL)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()

if ((TEST) AND (NOT ALL))
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)

    add_executable(test_log
        test/test_log.cc
    )
    target_link_libraries(test_log
        log
        opencv_imgcodecs
    )

    add_executable(test_log_performance
        test/test_log_performance.cc
    )
    target_link_libraries(test_log_performance
        log
    )
endif()
