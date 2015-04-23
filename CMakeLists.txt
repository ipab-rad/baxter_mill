cmake_minimum_required(VERSION 2.8.3)
project(baxter_mill)

find_package(catkin
  REQUIRED
  rospy
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  baxter_core_msgs
  baxter_interface
  message_generation
)

add_service_files(
  FILES
  BaxterCommand.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  baxter_core_msgs
  baxter_interface
  message_runtime
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
