
add_to_project_include_dirs(${CMAKE_CURRENT_SOURCE_DIR})

include_directories( costmap_2d )
add_subdirectory(costmap_2d)

# subdirlist(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})
# foreach(subdir ${SUBDIRS})
#     include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/${subdir} )
#     add_subdirectory(${subdir})
# endforeach()
