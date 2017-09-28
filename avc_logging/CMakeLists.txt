cmake_minimum_required(VERSION 2.8.3)
project(avc_logging)

find_package(Boost REQUIRED COMPONENTS system)
find_package( OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs 
  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB writer_SRC
    "src/writer/*.h"
    "src/writer/*.cpp"
)

add_executable(writer ${writer_SRC})

 target_link_libraries(writer
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
)



 target_link_libraries(writer
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)
