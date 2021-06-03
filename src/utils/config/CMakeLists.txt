message("cmake config module...")

include_directories(
    include/config
)

add_library(config SHARED
    src/config.cpp
    src/config_manager.cpp
)

target_link_libraries(config
    opencv_core
)

if (NOT ALL)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()

if ((TEST) AND (NOT ALL))
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)

    add_executable(test_config
        test/test_config.cpp
    )
    target_link_libraries(test_config
        config
    )
endif()