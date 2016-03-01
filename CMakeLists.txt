cmake_minimum_required(VERSION 2.8.3)
project(zed_cv_driver)

find_package(catkin REQUIRED COMPONENTS
image_transport roscpp cv_bridge sensor_msgs nodelet camera_info_manager rostest roslint dynamic_reconfigure diagnostic_updater)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)

option( USE_ZED_SDK "Use ZED SDK" OFF )

if(USE_ZED_SDK)
    find_package(ZED 0.9 REQUIRED)
    find_package(CUDA 6.5 REQUIRED)
endif()


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

  add_definitions(-std=c++11)# -m64) #-Wall)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

if(USE_ZED_SDK)
    add_definitions(-DUSE_ZED_SDK=1)
include_directories(
  ${ZED_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  )
endif()

add_library(zed_cv_nodelet src/zed_cv_nodelet.cpp)
target_link_libraries(zed_cv_nodelet ${catkin_LIBRARIES})
if(USE_ZED_SDK)
target_link_libraries(zed_cv_nodelet ${ZED_LIBRARIES})
endif()

## Declare a cpp executable
add_executable(zed_camera_node src/zed_cv_node.cpp)
target_link_libraries(zed_camera_node ${catkin_LIBRARIES})
if(USE_ZED_SDK)
target_link_libraries(zed_camera_node ${ZED_LIBRARIES})
endif()
