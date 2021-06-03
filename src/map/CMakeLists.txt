message("cmake map...")

set (MAP_PATH ${PROJECT_SOURCE_DIR}/src/map)

include_directories(
    include
    src
)

if (CLEAN_MAP OR ALL)
    if (EXISTS "${MAP_PATH}/clean_map/CMakeLists.txt")
        add_subdirectory(clean_map)
    endif()
endif()

if (LOCAL_MAP OR ALL)
    if (EXISTS "${MAP_PATH}/local_map/CMakeLists.txt")
        add_subdirectory(local_map)
    endif()
endif()