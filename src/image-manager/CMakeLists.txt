cmake_minimum_required(VERSION 3.0.2)
project(image_manager)
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++17 )
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs cv_bridge
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(image_manager src/image_manager_node.cpp src/image_manager.cpp src/base64.cpp)

target_link_libraries(image_manager
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)