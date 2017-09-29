cmake_minimum_required(VERSION 2.8.3)
project(avc_vision)

find_package(Boost REQUIRED COMPONENTS system)
find_package( OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  camera_info_manager
  pcl_ros  
)

file(GLOB msgs_to_mat_SRC
    "src/msgs_to_mat/*.h"
    "src/msgs_to_mat/*.cpp"
)

file(GLOB obj_detection_SRC
    "src/object_detection/*.h"
    "src/object_detection/*.cpp"
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs 
    camera_info_manager
  DEPENDS system_lib
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(msgs_to_mat ${msgs_to_mat_SRC})
add_executable(object_detection ${obj_detection_SRC})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(msgs_to_mat PUBLIC
  src/msgs_to_mat
 )

 target_link_libraries(msgs_to_mat
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)
 target_link_libraries(object_detection
   ${catkin_LIBRARIES}
)
