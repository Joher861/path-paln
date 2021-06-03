message("cmake task...")

include_directories(
    include/task
    include/task/include
    src
)

add_library(task SHARED
    src/task_manager.cc
)

target_link_libraries(task
    pthread
    event_center
    config
    log
)

if (NOT ALL)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()

if ((TEST) AND (NOT ALL))
    message("cmake test task...")
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)

    add_executable(test_task
        test/test_task_manager.cc
    )
    target_link_libraries(test_task
        task
    )
endif()
