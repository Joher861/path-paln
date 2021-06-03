# cmake_minimum_required(VERSION 2.8)
# project(teb_planner)

message( STATUS "cmake teb_planner...")

#########################################################
# version
set(PMS_TEB_VERSION_MAJOR 0)
set(PMS_TEB_VERSION_MINOR 1)
set(PMS_TEB_VERSION_PATCH 0)
set(PMS_TEB_VERSION ${PMS_TEB_VERSION_MAJOR}.${PMS_TEB_VERSION_MINOR}.${PMS_TEB_VERSION_PATCH})


# add -fPIC
# add_compile_options(-fPIC)
#or
#set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#####################################################
# add cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/3rdparty/g2o/cmake_modules")
set(SDK_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
# include cmake file
include(project_base)
include(install_package)

SET( G2O_ROOT ${PROJECT_SOURCE_DIR}/3rdparty/g2o )
FIND_PACKAGE( G2O )


INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR})
SET( CSPARSE_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rdparty/g2o/include/g2o/EXTERNAL/csparse)
INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})


# option
option( BUILD_SHARED_LIBS "Build shared libraries." ON)
option( BUILD_G2O_SHARED_LIBS "Build g2o shared libraries." OFF)

link_directories (
    # ${PROJECT_SOURCE_DIR}/3rdparty/boost/lib/${BUILD_PLATFORM}
    ${PROJECT_SOURCE_DIR}/3rdparty/linux_ipc_wrappers/lib/${BUILD_PLATFORM}
    ${PROJECT_SOURCE_DIR}/3rdparty/g2o/lib/${BUILD_PLATFORM}
    
)


if(BUILD_G2O_SHARED_LIBS)
    add_subdirectory(third_party/g2o)
else(BUILD_G2O_SHARED_LIBS)
    add_subdirectory(include/teb_planner)
    add_subdirectory(src)
    add_subdirectory(third_party)
endif(BUILD_G2O_SHARED_LIBS)




get_property( INTERNAL_INC  GLOBAL PROPERTY PROJECT_INCLUDE_DIRS )
get_property( INTERNAL_LIBS GLOBAL PROPERTY PROJECT_LIBRARIES )
get_property( TEB_SOURCES GLOBAL PROPERTY PROJECT_SOURCES )
get_property( TEB_HEADERS GLOBAL PROPERTY PROJECT_HEADERS )


# include_directories(${Boost_INCLUDE_DIRS})
# include_directories(${INTERNAL_INC})

include_directories(${PROJECT_SOURCE_DIR}/3rdparty/g2o/include)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/g2o/include/EXTERNAL/csparse)
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/linux_ipc_wrappers/)

include_directories(
    include
    include/teb_planner
    src
)

#shared library
if(BUILD_SHARED_LIBS)

    if(BUILD_G2O_SHARED_LIBS)
        set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/3rdparty/g2o/lib/${BUILD_PLATFORM})
        project_add_library(g2o SHARED ${TEB_SOURCES} ${TEB_HEADERS} )
    else(BUILD_G2O_SHARED_LIBS)
        if (NOT ALL)
            set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
        endif()
        project_add_library(teb_planner SHARED ${TEB_SOURCES} ${TEB_HEADERS} )
    endif(BUILD_G2O_SHARED_LIBS)
else()
    project_add_library(teb_planner_static STATIC ${TEB_SOURCES} ${TEB_HEADERS} )
    SET_TARGET_PROPERTIES(teb_planner_static PROPERTIES OUTPUT_NAME "teb_planner")
endif()

if(BUILD_G2O_SHARED_LIBS)

    target_link_libraries(g2o
        boost_thread
        boost_system
        g2o_ext_csparse
        
    )
else(BUILD_G2O_SHARED_LIBS)

    target_link_libraries(teb_planner 
                        ${INTERNAL_LIBS}
                        g2o_ext_csparse
                        g2o_core
                        g2o_csparse_extension
                        g2o_stuff
                        data_center
                        event_center
                        geometry
                        opencv_core
                        opencv_imgcodecs
                        opencv_imgproc
                        hsm
                        log
                        config
                        speed_controller
                        task
                        timer
                        linux_ipc_wrappers
                        boost_thread
                        boost_system
                        boost_graph
                        local_map
    )

endif(BUILD_G2O_SHARED_LIBS)



if ((TEST) AND (NOT ALL))

    if(BUILD_G2O_SHARED_LIBS)

    else(BUILD_G2O_SHARED_LIBS)
        add_subdirectory(test)
    endif(BUILD_G2O_SHARED_LIBS)

endif()
