cmake_minimum_required(VERSION 2.8.3)
project(thin_obj_bin_picking)

add_compile_options(-std=c++11)
add_definitions( -DROS_BUILD )

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  cmake_modules
  geometric_shapes
  hardware_interface
  controller_manager
  actionlib
  control_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  ur_msgs
  tf
  message_generation
  cv_bridge
)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package( OpenCV 3 REQUIRED )

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  img_status.msg
  blister_pose.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std-msgs hardware_interface controller_manager actionlib control_msgs geometry_msgs sensor_msgs trajectory_msgs ur_msgs message_runtime
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

