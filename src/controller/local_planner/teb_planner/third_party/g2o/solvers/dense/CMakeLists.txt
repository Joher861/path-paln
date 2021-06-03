
add_to_project_include_dirs(${CMAKE_CURRENT_SOURCE_DIR})
aux_include_directory(. HDRS)

add_to_project_headers(${HDRS})


if(BUILD_G2O_SHARED_LIBS)
    aux_src_directory(. SRCS)
    add_to_project_sources(${SRCS})
endif(BUILD_G2O_SHARED_LIBS)