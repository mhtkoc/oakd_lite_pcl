cmake_minimum_required(VERSION 3.8)
project(oakd_lite_pcl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# DepthAI C++ library - build path'i belirt
set(depthai_DIR "/home/pi/depthai-core/build")
find_package(depthai CONFIG REQUIRED)

add_executable(oakd_lite_pcl src/oakd_lite_pcl.cpp)

# LaserScan node
add_executable(laserscan_cpp src/oakd_lite_laserscan.cpp)

target_include_directories(oakd_lite_pcl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})

  target_include_directories(laserscan_cpp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS})# ROS dependencies first
ament_target_dependencies(oakd_lite_pcl
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions)

ament_target_dependencies(laserscan_cpp
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions)

# Then external libraries - plain signature (no PRIVATE keyword)
target_link_libraries(oakd_lite_pcl 
  depthai::core
  depthai::opencv
  ${PCL_LIBRARIES}
  Eigen3::Eigen)

target_link_libraries(laserscan_cpp 
  depthai::core
  depthai::opencv
  ${PCL_LIBRARIES})
    
install(TARGETS oakd_lite_pcl
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS laserscan_cpp
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()