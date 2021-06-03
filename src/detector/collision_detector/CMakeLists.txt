message("cmake collision detector...")

include_directories(
    include/collision_detector
    src
)

add_library(collision_detector SHARED
    src/collision_detector.cc
)

target_link_libraries(collision_detector
    pthread
    data_center
    event_center
    geometry
    hsm
    log
    timer
    config
)

if (NOT ALL)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()

if ((TEST) AND (NOT ALL))
    message("cmake test collision_detector...")
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)

    add_executable(collision_detector
        test/collision_detector.cc
    )
    target_link_libraries(collision_detector
        collision_detector
    )
endif()
