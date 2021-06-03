add_subdirectory(pcg)
add_subdirectory(dense)
add_subdirectory(structure_only)

add_subdirectory(csparse)

add_subdirectory(eigen)

add_to_project_include_dirs(${CMAKE_CURRENT_SOURCE_DIR})
