cmake_minimum_required(VERSION 3.14)
project(arise_slam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# FetchContent for dependencies not available via find_package
include(FetchContent)

# Dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

# glog / gflags
find_package(glog REQUIRED)
find_package(gflags REQUIRED)

# TBB
find_package(TBB REQUIRED)

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
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/arise_slam_mid360
    ${DIMOS_COMMON_DIR}
    ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${LCM_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
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
    src/FeatureExtraction/featureExtraction.cpp
    src/FeatureExtraction/LidarKeypointExtractor.cpp
    src/FeatureExtraction/DepthImageKeypointExtractor.cpp
    src/LaserMapping/laserMapping.cpp
    src/LaserMapping/lidarOptimization.cpp
    src/LaserMapping/LocalMap.cpp
    src/ImuPreintegration/imuPreintegration.cpp
    src/LidarProcess/LidarSlam.cpp
    src/LidarProcess/RollingGrid.cpp
    src/LidarProcess/pose_local_parameterization.cpp
    src/LidarProcess/SE3AbsolutatePoseFactor.cpp
    src/parameter/parameter.cpp
)

# Main executable
add_executable(arise_slam ${SOURCES})

target_link_libraries(arise_slam
    ${LCM_LIBRARIES}
    ${PCL_LIBRARIES}
    Eigen3::Eigen
    gtsam
    Ceres::ceres
    Sophus::Sophus
    glog::glog
    gflags
    TBB::tbb
    ${Boost_LIBRARIES}
    pthread
    stdc++fs
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(arise_slam OpenMP::OpenMP_CXX)
endif()

# Install
install(TARGETS arise_slam DESTINATION bin)
