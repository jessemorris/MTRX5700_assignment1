cmake_minimum_required(VERSION 2.8.3)
project(assignment_1)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	COMPONENTS
 	rospy 
 	interactive_markers
    pluginlib
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    gazebo_msgs
    moveit_visual_tools
    geometric_shapes
    std_msgs
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    )

catkin_python_setup()

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

## catkin specific configuration
catkin_package(
#  INCLUDE_DIRS include
  #LIBRARIES plotting
  CATKIN_DEPENDS rospy std_msgs message_runtime
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
    scripts/build_tower.py
  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
