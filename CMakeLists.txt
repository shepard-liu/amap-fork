#################################################
#
# amap CMakeLists.txt
#
#
# -- Build and Install(under the project root directory)
#
# 1. configure and generate with
#
#   cmake -Bbuild -Wno-dev . \
#       -G "Visual Studio 17 2022" \
#       -DCMAKE_INSTALL_PREFIX="C:/Program Files/amap-1.0" \
#       -DBUILD_EXAMPLES=ON
#  
# see "configuration options" section for more options available.
#
# 2. build with
#
#   cmake --build ./build --config Release
#
# 3. install with
#
#   cmake --install ./build --config Release
# 
#
#
# -- For Downstream Projects
#
# 1. find the amap package
#
# if you installed amap to the default install path(or other standard
# locations depending on your platform), you can find the amap package
# project with
# 
#  find_package(amap 1.0)
#
# otherwise, you need to specify the amap package project directory
# with
# 
#  find_package(amap 1.0 PATHS "path/to/amap/config.cmake")
#
# you may also find it by setting the CMAKE_PREFIX_PATH variable
#
# 2. link the amap library
#
#  target_link_libraries("[Your Target]" amap::amap)
#
# Eigen3 is also required to be found by the downstream project.
#
#################################################

cmake_minimum_required(VERSION 3.23 FATAL_ERROR)
include(CMakePackageConfigHelpers)

project(amap
    VERSION     1.0
    LANGUAGES   CXX
)

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

# macro to define options of variable types
macro(define_option option default type docstring)
    if(NOT DEFINED ${option})
        set(${option} ${default})
    endif()
    set(${option} ${${option}} CACHE ${type} ${docstring} FORCE)
endmacro(define)


# cxx_executable_with_flags(name cxx_flags libs srcs...)
#
# Creates a named C++ executable that depends on the given libraries and
# is built from the given source files.
macro(cxx_executable name libs)
    add_executable(${name} ${ARGN})
    set_target_properties(${name}
        PROPERTIES
            CXX_STANDARD 14
            CXX_STANDARD_REQUIRED YES
            CXX_EXTENSIONS NO
            POSITION_INDEPENDENT_CODE 1
    )

    if(cxx_flags)
        set_target_properties(${name}
            PROPERTIES
                COMPILE_FLAGS ${cxx_flags}
        )
    endif()

    foreach(lib ${libs})
        target_link_libraries(${name} ${lib})
    endforeach()

    # works for all build systems
    target_compile_definitions(${name} PRIVATE $<$<CONFIG:Debug>:DEBUGMODE>)

    list(APPEND BINARY_TARGETS ${name})
endmacro()

#-----------------------------------
# configuration options
#-----------------------------------

define_option(LIBRARY_FILES_INSTALL_DIR "lib" STRING "Library files install directory(under CMAKE_INSTALL_PREFIX)")
define_option(CONFIG_FILES_INSTALL_DIR "${LIBRARY_FILES_INSTALL_DIR}/cmake/${PROJECT_NAME}" STRING "Config files install directory(under CMAKE_INSTALL_PREFIX)")
define_option(BINARY_FILES_INSTALL_DIR "bin" STRING "Binary files install directory(under CMAKE_INSTALL_PREFIX)")
define_option(HEADER_FILES_INSTALL_DIR "include" STRING "Header files install directory")
define_option(BUILD_EXAMPLES OFF "Build example programs" BOOL)

set(CONFIG_FILE_NAME "${PROJECT_NAME}Config.cmake")
set(TARGETS_FILE_NAME "${PROJECT_NAME}Targets.cmake")
set(VERSION_FILE_NAME "${PROJECT_NAME}ConfigVersion.cmake")
set(DEFAULT_BUILD_TYPE "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES) # CMAKE_BUILD_TYPE does not work with multi-config generators like MSVC, Xcode, etc.
    message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE
            STRING "Choose the type of build." FORCE)
endif()

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "$ENV{ProgramFiles\(x86\)}/${PROJECT_NAME}-${CMAKE_PROJECT_VERSION}" CACHE PATH "Default install path" FORCE)
endif()


#-----------------------------------
# project dependencies
#-----------------------------------

find_package(Eigen3 REQUIRED QUIET)

#-----------------------------------
# amap header-only library
#-----------------------------------

# add a header-only library for downstream projects to consume
add_library(amap INTERFACE)
target_link_libraries(amap INTERFACE Eigen3::Eigen)

file(GLOB_RECURSE PUBLIC_HEADERS_FILES include/*)
target_sources(amap
    INTERFACE
    FILE_SET    HEADERS
    BASE_DIRS   include   
    FILES       ${PUBLIC_HEADERS_FILES}
)

#-----------------------------------
# example programs
#-----------------------------------

if(BUILD_EXAMPLES)
    cxx_executable(read_amap amap src/read_amap.cpp)
    cxx_executable(write_amap amap src/write_amap.cpp)
    cxx_executable(update_amap amap src/update_amap.cpp)
    cxx_executable(pointcloud_io amap src/pointcloud_io.cpp)
    cxx_executable(pointcloud_filter amap src/pointcloud_filter.cpp)
    cxx_executable(read_amap_frameposes amap src/read_amap_frameposes.cpp)
    cxx_executable(amap_header_info amap src/amap_header_info.cpp)
endif()

#-----------------------------------
# project installation
#-----------------------------------

install(
    TARGETS     amap
    EXPORT      amapTargets
    FILE_SET    HEADERS
    DESTINATION "${HEADER_FILES_INSTALL_DIR}/amap"
)

install(
    EXPORT      amapTargets
    FILE        ${TARGETS_FILE_NAME}
    NAMESPACE   amap::
    DESTINATION ${CONFIG_FILES_INSTALL_DIR}
)

write_basic_package_version_file(
    ${VERSION_FILE_NAME}
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    "cmake/amapConfig.cmake.in"
    ${CONFIG_FILE_NAME}
    PATH_VARS
        HEADER_FILES_INSTALL_DIR
        LIBRARY_FILES_INSTALL_DIR
        BINARY_FILES_INSTALL_DIR
    INSTALL_DESTINATION ${CONFIG_FILES_INSTALL_DIR}
)

install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/${CONFIG_FILE_NAME}"
        "${CMAKE_CURRENT_BINARY_DIR}/${VERSION_FILE_NAME}"
    DESTINATION
        ${CONFIG_FILES_INSTALL_DIR}
)

if(BUILD_EXAMPLES)
    install(
        TARGETS ${BINARY_TARGETS}
        RUNTIME DESTINATION "${BINARY_FILES_INSTALL_DIR}/examples"
    )
endif()

message(STATUS "BUILD_EXAMPLES: ${BUILD_EXAMPLES}")
message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "CONFIG_FILES_INSTALL_DIR: ${CONFIG_FILES_INSTALL_DIR}")
message(STATUS "LIBRARY_FILES_INSTALL_DIR: ${LIBRARY_FILES_INSTALL_DIR}")
message(STATUS "BINARY_FILES_INSTALL_DIR: ${BINARY_FILES_INSTALL_DIR}")
message(STATUS "HEADER_FILES_INSTALL_DIR: ${HEADER_FILES_INSTALL_DIR}")