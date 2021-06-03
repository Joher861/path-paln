
add_subdirectory(stuff)
add_subdirectory(core)

add_subdirectory(EXTERNAL)
# Solvers
add_subdirectory(solvers)


#include_directories(.)
add_to_project_include_dirs(${CMAKE_CURRENT_SOURCE_DIR})


aux_include_directory(. HDRS)
add_to_project_headers(${HDRS})
