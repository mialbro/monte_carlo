cmake_minimum_required(VERSION 3.0.2)
project(monte_carlo)

add_compile_options(-std=c++14)

set(OpenCV_DIR ~/opencv4.5/opencv/build/)
find_package(OpenCV 4.5 REQUIRED)

find_package(PkgConfig)
find_package(catkin REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  rosbag
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${DBoW2_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
)

add_library(${PROJECT_NAME}_node
    src/measurement_model.cpp
    src/particle_filter.cpp
    src/velocity_motion_model.cpp
    src/monte_carlo_localization.cpp
)

add_executable(${PROJECT_NAME}_ex
  src/monte_carlo_localization.cpp
)
target_link_libraries(${PROJECT_NAME}_ex
  ${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(controller
    src/controller.cpp
)
 target_link_libraries(controller
 ${OpenCV_LIBS}
 ${catkin_LIBRARIES})
