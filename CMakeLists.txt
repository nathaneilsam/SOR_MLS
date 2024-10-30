cmake_minimum_required(VERSION 3.0)

project(SOR_MLS_modular)

SET(CMAKE_CXX_STANDARD 17)
# set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic")
# SET(CMAKE_CXX_FLAGS "-std=c++14 -w -fopenmp -pthread ")
#SET(CMAKE_C_FLAGS "-O3 -std=c++14 -fopenmp -pthread ")

SET(CMAKE_BUILD_TYPE "Release")#"Debug")#
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
# SET(SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/CSF/src)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})#/bin)
#SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
# include_directories("${CMAKE_CURRENT_SOURCE_DIR}/CSF/include")
####### Required libs #######
#eigen3, liblas, OpenCV (>=2.4), pcl (>=1.7)
find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
#BOOST
find_package(Boost COMPONENTS thread serialization timer chrono filesystem date_time system REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})
add_definitions(-DBOOST_ALL_DYN_LINK)

# set(TBB_DIR "/usr/local/tbb-2019_U8")
# include("/home/nathaniel/threepart/oneTBB-2019_U8/cmake/TBBBuild.cmake")
# tbb_build(TBB_ROOT "/home/nathaniel/threepart/oneTBB-2019_U8" CONFIG_DIR TBB_DIR)
# # set(TBB_ROOT "/usr/local/")
# find_package(TBB REQUIRED)
# if (TBB_FOUND)
#     include_directories(${TBB_INCLUDE_DIRS})
#     add_definitions(-DCGAL_LINKED_WITH_TBB)
#     # target_link_libraries(your_target_name ${TBB_LIBRARIES})
# else()
#     message(FATAL_ERROR "TBB not found!")
# endif()
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("DEP: EIGEN [OK] ")

# find_package(glog REQUIRED)
#PCL1.8
find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# find_package(CGAL REQUIRED)
# include_directories("/usr/include/CGAL")
# target_link_libraries(your_project CGAL::CGAL)

# 查找 libproj 库
find_library(PROJ_LIB proj HINTS "/usr/lib" "/usr/lib/x86_64-linux-gnu" "/usr/local/lib")
# 检查是否找到库
if(NOT PROJ_LIB)
    message(FATAL_ERROR "libproj not found!")
endif()


SET(INCLUDE_DIR "include")
# SET(LIBPCL_HANDLE "/home/nathaniel/桌面/xingshen/pcl_handle_so")
include_directories("${INCLUDE_DIR}")
set(SOURCE_FILES
    src/AxisTran.cpp
    src/pcl_function.cpp
    src/xsproto_common_common.cpp
    src/buildSpatialMap.cpp
)
# message(STATUS "TBB_INCLUDE_DIRS: ${TBB_INCLUDE_DIRS}")
# message(STATUS "TBB_LIBRARIES: ${TBB_LIBRARIES}")
# link_directories("${LIBPCL_HANDLE}")
add_executable(SOR_MLS_modular main.cpp ${SOURCE_FILES})
target_link_libraries(SOR_MLS_modular ${Boost_LIBRARIES} ${Eigen3_LIBRARIES} ${PROJ_LIB} ${PCL_LIBRARIES} -lpthread)#TBB::tbb CGAL::CGAL)# pcl_handle)