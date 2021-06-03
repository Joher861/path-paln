message("cmake event_center...")

include_directories(
    include/event_center
    src
)

add_library(event_center SHARED
    src/event_center.cc
)

target_link_libraries(event_center
    log
    timer
    config
)

if (NOT ALL)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()

if ((TEST) AND (NOT ALL))
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)

    add_executable(test_event_center
        test/test_event_center.cc
    )
    target_link_libraries(test_event_center
        event_center
    )
endif()
