cmake_minimum_required(VERSION 2.8.3)
project(zed_cv_driver)

find_package(catkin REQUIRED COMPONENTS
image_transport roscpp cv_bridge sensor_msgs nodelet camera_info_manager rostest roslint dynamic_reconfigure diagnostic_updater)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)

generate_dynamic_reconfigure_options(
  cfg/Zed.cfg
)

roslint_cpp()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES zed_cv_driver
  CATKIN_DEPENDS image_transport roscpp cv_bridge sensor_msgs nodelet camera_info_manager
  DEPENDS opencv2
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )

add_library(zed_cv_nodelet src/zed_cv_nodelet.cpp)
target_link_libraries(zed_cv_nodelet ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(zed_camera_node src/zed_cv_node.cpp)
target_link_libraries(zed_camera_node ${catkin_LIBRARIES})
