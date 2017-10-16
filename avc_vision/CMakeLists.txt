cmake_minimum_required(VERSION 2.8.3)
project(avc_vision)

add_definitions(-std=c++11)

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
  tf
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/cam.cfg
)

file(GLOB curve_center_SRC
    "src/curve_center/*.h"
    "src/curve_center/*.cpp"
)

file(GLOB side_obj_filter_SRC
    "src/side_obj_filter/*.h"
    "src/side_obj_filter/*.cpp"
)

file(GLOB obj_detection_SRC
    "src/object_detection/*.h"
    "src/object_detection/*.cpp"
)
file(GLOB ocam_SRC
  "src/ocam/*.h"  
  "src/ocam/*.cpp"
)
file(GLOB front_obj_SRC
"src/front_obj_filter/*.h"  
"src/front_obj_filter/*.cpp"
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs 
    camera_info_manager tf dynamic_reconfigure pcl_ros
  DEPENDS system_lib
)

add_executable(side_obj_filter ${side_obj_filter_SRC})
add_executable(object_detection ${obj_detection_SRC})
add_executable(ocam ${ocam_SRC})
add_executable(front_obj_filter ${front_obj_SRC})
add_executable(curve_center ${curve_center_SRC})

target_link_libraries(ocam ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} -lv4l2 -ludev)

include_directories(
  include/ocam
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


 target_link_libraries(side_obj_filter
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
 target_link_libraries(object_detection
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
target_link_libraries(front_obj_filter
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)
target_link_libraries(curve_center
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)
add_dependencies(ocam ${PROJECT_NAME}_gencfg)
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})
