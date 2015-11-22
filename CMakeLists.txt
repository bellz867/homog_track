cmake_minimum_required(VERSION 2.8.3)
project(homog_track)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_geometry
  image_transport
  tf
  message_generation
  rosbag
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

#catkin_package(  ...
  #CATKIN_DEPENDS message_runtime ...
  #...))

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_message_files(
  FILES
  HomogMarker.msg
  HomogComplete.msg
  HomogDecomposed.msg
  HomogDesired.msg
  ImageProcessingMsg.msg
  DecompMsg.msg
)

add_service_files(
  FILES
  HomogReference.srv
  CameraDesiredStart.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare a C++ executable
add_executable(tracking_node src/marker_track_5.cpp)
target_link_libraries(tracking_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(tracking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(decomp_node src/homog_decompose_2.cpp)
target_link_libraries(decomp_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(decomp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(desired_camera_node src/camera_desired_4.cpp)
target_link_libraries(desired_camera_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(desired_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(quad_controller_node src/quad_controller_3.cpp)
target_link_libraries(quad_controller_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(quad_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(simulator_node src/simulator_with_nodes.cpp)
target_link_libraries(simulator_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(simulator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(nodetastic src/testing_stuff.cpp)
target_link_libraries(nodetastic ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(nodetastic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(experiment src/experiment.cpp)
target_link_libraries(experiment ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(experiment ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(threshold_testing_node src/camera_threshold.cpp)
target_link_libraries(threshold_testing_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(threshold_testing_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
