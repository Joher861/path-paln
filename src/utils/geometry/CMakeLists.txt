message("cmake geometry...")

include_directories(
    include/geometry
)

add_library(geometry SHARED
    src/grid.cc
    src/point.cc
    src/pose.cc
    src/line.cc
    src/trajectory.cc
    src/rect.cc
    src/grid_rect.cc
    src/path.cc
    src/contour.cc
    src/area.cc
    src/geometry_func.cc
)

if (NOT ALL)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()

if ((TEST) AND (NOT ALL))
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)

    add_executable(test_geometry
        test/test_geometry.cc
    )
    target_link_libraries(test_geometry
        geometry
        opencv_core
        opencv_imgcodecs
        opencv_imgproc
    )
endif()
