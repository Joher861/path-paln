message("cmake planner...")

set (PLANNER_PATH ${PROJECT_SOURCE_DIR}/src/planner)

include_directories(
    include
)

if (NAVIGATION_PLANNER OR ALL)
    if (EXISTS "${PLANNER_PATH}/navigation_planner/CMakeLists.txt")
        add_subdirectory(navigation_planner)
    endif()
endif()

if (CCPP_COVERAGE_PLANNER OR ALL)
    if (EXISTS "${PLANNER_PATH}/ccpp_coverage_planner/CMakeLists.txt")
        add_subdirectory(ccpp_coverage_planner)
    endif()
endif()

if (PATH_FOLLOW_PLANNER OR ALL)
    if (EXISTS "${PLANNER_PATH}/path_follow_planner/CMakeLists.txt")
        add_subdirectory(path_follow_planner)
    endif()
endif()

if (GLOBAL_TEB_PLANNER OR ALL)
    if (EXISTS "${PLANNER_PATH}/teb_planner/CMakeLists.txt")
        add_subdirectory(teb_planner)
    endif()
endif()
