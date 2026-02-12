cmake_minimum_required(VERSION 3.14)
project(livox_mid360_native CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fetch dimos-lcm for C++ message headers
include(FetchContent)
FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG main
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

# Find LCM
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

# Livox SDK2 (installed to /usr/local)
find_library(LIVOX_SDK livox_lidar_sdk_shared HINTS /usr/local/lib)
if(NOT LIVOX_SDK)
  message(FATAL_ERROR "Livox SDK2 not found. Install from https://github.com/Livox-SDK/Livox-SDK2")
endif()

add_executable(mid360_native main.cpp)

target_include_directories(mid360_native PRIVATE
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  /usr/local/include  # Livox SDK2 headers
)

target_link_libraries(mid360_native PRIVATE
  ${LCM_LIBRARIES}
  ${LIVOX_SDK}
)

target_link_directories(mid360_native PRIVATE
  ${LCM_LIBRARY_DIRS}
)
