cmake_minimum_required(VERSION 3.20)
project(fastlivo2)

set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set common compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

# Detect CPU architecture
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# Specific settings for Release build
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -ffast-math")
  message(STATUS "Using ARM optimizations")
  add_definitions(-DARM_ARCH)
else()
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops")
  message(STATUS "Using x86 optimizations")
  add_definitions(-DX86_ARCH)
endif()

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(vikit_common REQUIRED)
find_package(livox_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenMP REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  pcl_ros
  tf2_ros
  cv_bridge
  image_transport
  Eigen3
  PCL
  OpenCV
  Sophus
  Boost
  vikit_common
  livox_interfaces
  visualization_msgs
  OpenMP
)

# Include directories
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  include
)

# # set Sourcefiles
# LIST(APPEND SOURCEFILES src/atan_camera.cpp
#                         src/omni_camera.cpp
#                         src/math_utils.cpp
#                         src/vision.cpp
#                         src/performance_monitor.cpp
#                         src/robust_cost.cpp
#                         src/user_input_thread.cpp
#                         src/pinhole_camera.cpp
#                         src/equidistant_camera.cpp
#                         src/polynomial_camera.cpp
#                         src/homography.cpp
#                         src/img_align.cpp)

set(Sophus_LIBRARIES libSophus.so)
    
# Link dependencies using ament
add_library(laser_mapping
  src/vio.cpp
  src/frame.cpp
  src/visual_point.cpp
  src/voxel_map.cpp
  src/preprocess.cpp
  src/IMU_Processing.cpp
  src/LIVMapper.cpp
)

ament_target_dependencies(laser_mapping
  ${dependencies}
)

add_executable(fastlivo_mapping
  src/main.cpp
)

target_link_libraries(fastlivo_mapping
  laser_mapping
  ${Sophus_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
)

install(
  TARGETS fastlivo_mapping
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(laser_mapping)
ament_export_dependencies(${dependencies})
ament_package()