cmake_minimum_required(VERSION 2.8.3)
project(mavros_controllers)

find_package(catkin REQUIRED COMPONENTS
  mavros
  roscpp
  rospy
  tf
  dynamic_reconfigure
  message_generation
)

catkin_python_setup()

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/mavros_controllers.cfg
  #cfg/DynReconf2.cfg
)


catkin_package(
  INCLUDE_DIRS 
#  LIBRARIES mavros_controllers
  CATKIN_DEPENDS mavros roscpp rospy tf message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

