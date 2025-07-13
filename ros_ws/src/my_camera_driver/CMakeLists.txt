cmake_minimum_required(VERSION 3.0.2)
project(my_camera_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ip_camera_publisher.py
  scripts/camera_overlay_node.py
  scripts/sensor_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
