cmake_minimum_required(VERSION 3.14)
project(better_fastlio2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    add_definitions(-DMP_EN)
    include(ProcessorCount)
    ProcessorCount(NPROC)
    if(NPROC GREATER 0)
        add_definitions(-DMP_PROC_NUM=${NPROC})
    else()
        add_definitions(-DMP_PROC_NUM=4)
    endif()
endif()

# FetchContent for dependencies not available via find_package
include(FetchContent)

# Dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration)
find_package(GTSAM REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

# LCM
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

# dimos-lcm message headers (fetched from GitHub during build)
if(DEFINED FETCHCONTENT_SOURCE_DIR_DIMOS_LCM)
    set(FETCHCONTENT_SOURCE_DIR_dimos_lcm ${FETCHCONTENT_SOURCE_DIR_DIMOS_LCM})
endif()
FetchContent_Declare(
    dimos_lcm
    GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
    GIT_TAG main
)
FetchContent_MakeAvailable(dimos_lcm)

# Include directories
set(COMMON_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

# DIMOS_COMMON_DIR: the NativeModule CLI helper header.
# In a nix build the relative path won't resolve, so fall back to
# the project source tree when the dir exists, otherwise expect
# the header to be provided via DIMOS_COMMON_DIR cmake flag.
set(_default_common_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../../hardware/sensors/lidar/common")
if(NOT DEFINED DIMOS_COMMON_DIR)
    if(EXISTS "${_default_common_dir}/dimos_native_module.hpp")
        set(DIMOS_COMMON_DIR "${_default_common_dir}")
    else()
        set(DIMOS_COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    endif()
endif()

include_directories(
    ${COMMON_INCLUDE_DIR}
    ${COMMON_INCLUDE_DIR}/IKFoM_toolkit
    ${DIMOS_COMMON_DIR}
    ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${LCM_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${LCM_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# Source files
set(SOURCES
    main.cpp
    preprocess.cpp
    ikd_Tree.cpp
    Scancontext.cpp
    tgrs.cpp
)

# Main executable
add_executable(better_fastlio2 ${SOURCES})

target_link_libraries(better_fastlio2
    ${LCM_LIBRARIES}
    ${PCL_LIBRARIES}
    Eigen3::Eigen
    gtsam
    ${Boost_LIBRARIES}
    pthread
    stdc++fs
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(better_fastlio2 OpenMP::OpenMP_CXX)
endif()

# Install
install(TARGETS better_fastlio2 DESTINATION bin)
