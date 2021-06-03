message("cmake detector...")

set (DETECTOR_PATH ${PROJECT_SOURCE_DIR}/src/detector)

include_directories(
    include
)

if (EMERGENCY_STOP_DETECTOR OR ALL)
    if (EXISTS "${DETECTOR_PATH}/emergency_stop_detector/CMakeLists.txt")
        add_subdirectory(emergency_stop_detector)
    endif()
endif()

if (OBSTACLE_DETECTOR OR ALL)
    if (EXISTS "${DETECTOR_PATH}/obstacle_detector/CMakeLists.txt")
        add_subdirectory(obstacle_detector)
    endif()
endif()

if (COLLISION_DETECTOR OR ALL)
    if (EXISTS "${DETECTOR_PATH}/collision_detector/CMakeLists.txt")
        add_subdirectory(collision_detector)
    endif()
endif()