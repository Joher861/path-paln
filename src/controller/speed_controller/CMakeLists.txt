message("cmake speed_controller...")

include_directories(
    include/speed_controller
    src
)

add_library(speed_controller SHARED
    src/speed_controller.cc
)

target_link_libraries(speed_controller
    pthread
    event_center
    config
    log
)

if (NOT ALL)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
endif()

if((TEST) AND (NOT ALL))
   message("cmake test speed_controller...")
   set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/bin)

   add_executable(test_speed_controller
       test/test_speed_controller.cc
   )
   target_link_libraries(test_speed_controller
       speed_controller
       opencv_core
       geometry
   )
endif()
