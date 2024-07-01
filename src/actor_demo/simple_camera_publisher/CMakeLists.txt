cmake_minimum_required(VERSION 2.8.3)
project(simple_camera_publisher)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES actor_input
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_executable(cam_pub src/cam_pub.cpp)
add_dependencies(cam_pub ${PROJECT_NAME}_gencfg)


target_link_libraries(cam_pub ${catkin_LIBRARIES} ${OpenCV_LIBS})
