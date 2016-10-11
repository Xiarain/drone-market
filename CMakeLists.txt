cmake_minimum_required(VERSION 2.8.3)

project(droneTest)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

#catkin_package()

find_package(OpenCV)

include_directories(
 ${OpenCV_INCLUDE_DIRS}
)

aux_source_directory(./src DIR_SRCS)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(droneTest src/main.cpp ${DIR_SRCS})
target_link_libraries(droneTest ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(droneTest droneTest_generate_messages_cpp)












