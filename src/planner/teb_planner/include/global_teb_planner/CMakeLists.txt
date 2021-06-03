

add_to_project_include_dirs(${CMAKE_CURRENT_SOURCE_DIR})
subdirlist(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})
foreach(subdir ${SUBDIRS})
    add_to_project_include_dirs( ${CMAKE_CURRENT_SOURCE_DIR}/${subdir} )
    add_subdirectory(${subdir})
endforeach()
