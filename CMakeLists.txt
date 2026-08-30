cmake_minimum_required(VERSION 2.8.3)
project(fast_livo)

set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set common compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

# Specific settings for Debug build
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

# Detect CPU architecture
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# Specific settings for Release build
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    # 64-bit ARM optimizations (e.g., RK3588 and Jetson Orin NX)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -ffast-math")
    message(STATUS "Using 64-bit ARM optimizations: -O3 -mcpu=native -mtune=native -ffast-math")
  else()
    # 32-bit ARM optimizations with NEON support
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
    message(STATUS "Using 32-bit ARM optimizations: -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
  endif()
  add_definitions(-DARM_ARCH)
else()
  # x86-64 (Intel/AMD) optimizations
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops") #-flto
  message(STATUS "Using general x86 optimizations: -O3 -march=native -mtune=native -funroll-loops") 
  add_definitions(-DX86_ARCH)
endif()

# Define project root directory
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Detect CPU core count for potential multithreading optimization
include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

# Set the number of cores for multithreading
if(N GREATER 4)
  math(EXPR PROC_NUM "4")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
elseif(N GREATER 1)
  math(EXPR PROC_NUM "${N}")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
else()
  add_definitions(-DMP_PROC_NUM=1)
  message(STATUS "Single core detected. Multithreading disabled.")
endif()

# Check for OpenMP support
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  add_compile_options(${OpenMP_CXX_FLAGS})
else()
  message(STATUS "OpenMP not found, proceeding without it")
endif()

# Check for mimalloc support
find_package(mimalloc QUIET)
if(mimalloc_FOUND)
  message(STATUS "mimalloc found")
else()
  message(STATUS "mimalloc not found, proceeding without it")
endif()

# Find catkin and required dependencies
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
  vikit_common
  vikit_ros
  cv_bridge
  image_transport
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

set(Sophus_LIBRARIES libSophus.so)

add_message_files(FILES Diagnostics.msg)
generate_messages(DEPENDENCIES std_msgs)

# Define the catkin package
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime cv_bridge vikit_common vikit_ros image_transport
  DEPENDS EIGEN3 PCL OpenCV Sophus
)

# Include directories for dependencies
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  include
)

# Add libraries
add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(lio src/voxel_map.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(laser_mapping src/LIVMapper.cpp)

add_dependencies(vio ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lio ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(laser_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Add the main executable
add_executable(fastlivo_mapping src/main.cpp)
add_dependencies(fastlivo_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Link libraries to the executable
target_link_libraries(fastlivo_mapping
  laser_mapping
  vio
  lio
  pre
  imu_proc
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Link mimalloc if found
if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping mimalloc)
endif()

# Official usage (https://koide3.github.io/iridescence/):
#   find_package(Iridescence REQUIRED)
#   target_link_libraries(... Iridescence::Iridescence)
# iridescence-config.cmake then does:
#   include(iridescence-targets.cmake)
#   add_library(Iridescence::Iridescence ALIAS Iridescence::iridescence)
# CMake 3.16 (Ubuntu 20.04 / Noetic) cannot ALIAS a non-GLOBAL imported target
# from a catkin subdirectory. Load the same files and make the import GLOBAL
# so the documented Iridescence::Iridescence name exists.
find_path(IRIDESCENCE_CMAKE_DIR iridescence-config.cmake
  HINTS
    ${CMAKE_PREFIX_PATH}
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../.local/iridescence"
    /usr/local
    /usr
  PATH_SUFFIXES lib/cmake/iridescence
)

if(IRIDESCENCE_CMAKE_DIR)
  include(CMakeFindDependencyMacro)
  find_dependency(OpenGL REQUIRED)
  find_dependency(Eigen3 REQUIRED)
  find_dependency(PNG)
  find_dependency(JPEG)
  include("${IRIDESCENCE_CMAKE_DIR}/iridescence-targets.cmake")
  set_property(TARGET Iridescence::iridescence PROPERTY IMPORTED_GLOBAL TRUE)
  if(NOT TARGET Iridescence::Iridescence)
    add_library(Iridescence::Iridescence ALIAS Iridescence::iridescence)
  endif()

  add_executable(fastlivo_viewer src/viewer.cpp)
  add_dependencies(fastlivo_viewer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  # GLIM default is BUILD_WITH_MARCH_NATIVE=OFF. LightViewer is `new`'d without
  # EIGEN_MAKE_ALIGNED_OPERATOR_NEW and stores Eigen::Matrix4f; AVX from the
  # package-wide -march=native faults in ShaderSetting.
  target_compile_options(fastlivo_viewer PRIVATE -mno-avx -mno-avx2 -mno-fma)
  target_link_libraries(fastlivo_viewer
    ${roscpp_LIBRARIES}
    ${cv_bridge_LIBRARIES}
    ${image_transport_LIBRARIES}
    ${pcl_conversions_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    opencv_core
    opencv_imgproc
    Iridescence::Iridescence
  )

  get_filename_component(_iri_prefix "${IRIDESCENCE_CMAKE_DIR}/../../.." ABSOLUTE)
  set_target_properties(fastlivo_viewer PROPERTIES
    BUILD_RPATH "${_iri_prefix}/lib"
    INSTALL_RPATH "${_iri_prefix}/lib"
  )
else()
  message(WARNING "Iridescence not found. fastlivo_viewer skipped. Install libiridescence-dev or build koide3/iridescence.")
endif()
