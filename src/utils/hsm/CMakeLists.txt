message("cmake state_machine...")

include_directories(
    include/hsm
)

add_library(hsm SHARED
    src/hsm.cc
)

target_link_libraries(hsm
    event_center
    pthread
    log
    timer
)

if (NOT ALL)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()

if ((TEST) AND (NOT ALL))
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)

    add_executable(test_hsm
        test/test_hsm.cc
        test/main.cc
    )
    target_link_libraries(test_hsm
        hsm
    )
endif()
