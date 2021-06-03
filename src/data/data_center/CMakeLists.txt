message("cmake data_center...")

include_directories(
    include/data_center
    include/data_center/include
    src
)

add_library(data_center SHARED
    src/data_center.cc
)

target_link_libraries(data_center
    log
    config
)

if (NOT ALL)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()

if ((TEST) AND (NOT ALL))
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)

    add_executable(test_data_center
        test/test_data_center.cc
    )
    target_link_libraries(test_data_center
        data_center
        pthread
    )
endif()
