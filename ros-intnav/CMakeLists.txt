cmake_minimum_required(VERSION 2.8.3)
project(duckietown-intnav)

find_package(catkin REQUIRED COMPONENTS 
    rospy
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    tf
)

catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

