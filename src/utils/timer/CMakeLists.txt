message("cmake timer...")

include_directories(
    include/timer
)

add_library(timer SHARED
    src/timer.cc
    src/sleep_timer.cc
)

target_link_libraries(timer
    pthread
)

if (NOT ALL)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()

if ((TEST) AND (NOT ALL))
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)

    add_executable(test_timer
        test/test_timer.cc
    )
    target_link_libraries(test_timer
        timer
    )
endif()
