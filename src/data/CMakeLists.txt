message("cmake data...")

set (DATA_PATH ${PROJECT_SOURCE_DIR}/src/data)

include_directories(
    include
)

if (EVENT_CENTER OR ALL)
    if (EXISTS "${DATA_PATH}/event_center/CMakeLists.txt")
        add_subdirectory(event_center)
    endif()
endif()

if (DATA_CENTER OR ALL)
    if (EXISTS "${DATA_PATH}/data_center/CMakeLists.txt")
        add_subdirectory(data_center)
    endif()
endif()