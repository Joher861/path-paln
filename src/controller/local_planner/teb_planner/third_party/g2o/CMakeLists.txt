
add_subdirectory(stuff)
add_subdirectory(core)

# add_subdirectory(EXTERNAL)
# Solvers
add_subdirectory(solvers)

option(G2O_FAST_MATH "Enable fast math operations" OFF)
option(G2O_NO_IMPLICIT_OWNERSHIP_OF_OBJECTS "Disables memory management in the graph types, this requires the callers to manager the memory of edges and nodes" OFF)
#include_directories(.)
add_to_project_include_dirs(${CMAKE_CURRENT_SOURCE_DIR})


aux_include_directory(. HDRS)
add_to_project_headers(${HDRS})
