message("cmake utils...")

set (UTILS_PATH ${PROJECT_SOURCE_DIR}/src/utils)

include_directories(
    include
)

if (CONFIG OR ALL)
    if (EXISTS "${UTILS_PATH}/config/CMakeLists.txt")
        add_subdirectory(config)
    endif()
endif()

if (GEOMETRY OR ALL)
    if (EXISTS "${UTILS_PATH}/geometry/CMakeLists.txt")
        add_subdirectory(geometry)
    endif()
endif()

if (TIMER OR ALL)
    if (EXISTS "${UTILS_PATH}/timer/CMakeLists.txt")
        add_subdirectory(timer)
    endif()
endif()

if (LOG OR ALL)
    if (EXISTS "${UTILS_PATH}/log/CMakeLists.txt")
        add_subdirectory(log)
    endif()
endif()

if (HSM OR ALL)
    if (EXISTS "${UTILS_PATH}/hsm/CMakeLists.txt")
        add_subdirectory(hsm)
    endif()
endif()


