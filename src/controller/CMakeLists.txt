message("cmake controller...")

set (CONTROLLER_PATH ${PROJECT_SOURCE_DIR}/src/controller)

include_directories(
    include
    local_planner/include
)

if (SPEED_CONTROLLER OR ALL)
    if (EXISTS "${CONTROLLER_PATH}/speed_controller/CMakeLists.txt")
        add_subdirectory(speed_controller)
    endif()
endif()

if (TASK OR ALL)
    if (EXISTS "${CONTROLLER_PATH}/task/CMakeLists.txt")
        add_subdirectory(task)
    endif()
endif()

if (ROTATE_PLANNER OR ALL)
    if (EXISTS "${CONTROLLER_PATH}/local_planner/rotate_planner/CMakeLists.txt")
        add_subdirectory(local_planner/rotate_planner)
    endif()
endif()

if (BASIC_PLANNER OR ALL)
    if (EXISTS "${CONTROLLER_PATH}/local_planner/basic_planner/CMakeLists.txt")
        add_subdirectory(local_planner/basic_planner)
    endif()
endif()

if (MPC_PLANNER OR ALL)
    if (EXISTS "${CONTROLLER_PATH}/local_planner/mpc_planner/CMakeLists.txt")
        add_subdirectory(local_planner/mpc_planner)
    endif()
endif()

if (PURE_PURSUIT_PLANNER)
    if (EXISTS "${CONTROLLER_PATH}/local_planner/pure_pursuit_planner/CMakeLists.txt")
        add_subdirectory(local_planner/pure_pursuit_planner)
    endif()
endif()

# if (TEB_PLANNER OR ALL)
#     if (EXISTS "${CONTROLLER_PATH}/local_planner/teb_planner/CMakeLists.txt")
#         add_subdirectory(local_planner/teb_planner)
#     endif()
# endif()

if (WALL_FOLLOW_PLANNER OR ALL)
    if (EXISTS "${CONTROLLER_PATH}/local_planner/wall_follow_planner/CMakeLists.txt")
        add_subdirectory(local_planner/wall_follow_planner)
    endif()
endif()

