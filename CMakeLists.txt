cmake_minimum_required(VERSION 3.13)

#####
# Enables project() command manages VERSION variables.
if(POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif(POLICY CMP0048)

if(POLICY CMP0071)
    cmake_policy(SET CMP0071 NEW)
endif(POLICY CMP0071)

if(POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW)
endif(POLICY CMP0072)

project(amap VERSION 1.0)

## Compile as C++14 
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)

if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
        add_definitions(-DDEBUGMODE)
        message("Build Type: Debug ")
else( CMAKE_BUILD_TYPE STREQUAL "Release" ) 
        message("Build Type: Release ")
endif()

# Eigen
if(WIN32)
    # set(Eigen3_DIR "path/to/install/eigen3/cmake")
    # for example
    set(Eigen3_DIR "D:/projects/ampal/lib_win/Eigen3/share/eigen3/cmake")
endif()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
  ${PROJECT_SOURCE_DIR}
  include 
)

# set bin folder
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# example
add_executable(read_amap src/read_amap.cpp)
add_executable(write_amap src/write_amap.cpp)
add_executable(update_amap src/update_amap.cpp)
add_executable(pointcloud_io src/pointcloud_io.cpp)
add_executable(pointcloud_filter src/pointcloud_filter.cpp)
add_executable(read_amap_frameposes src/read_amap_frameposes.cpp)
add_executable(amap_header_info src/amap_header_info.cpp)

