cmake_minimum_required(VERSION 2.8.3)
project(extract_rgbd)
if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
)

catkin_package()

find_package( OpenCV REQUIRED )
include_directories( ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})



# ____________________________________
# |
# |               RGB
# |___________________________________

SET(SOURCES src/node_rgb.cpp)

add_executable(extract_rgb ${SOURCES})
target_link_libraries(extract_rgb ${catkin_LIBRARIES})
# ____________________________________
# |
# |               Depth
# |___________________________________

SET(SOURCES src/node_d.cpp)

add_executable(extract_d ${SOURCES})
target_link_libraries(extract_d ${catkin_LIBRARIES})
# ____________________________________
# |
# |               Depth 8m
# |___________________________________

SET(SOURCES src/node_d8.cpp)

add_executable(extract_d8 ${SOURCES})
target_link_libraries(extract_d8 ${catkin_LIBRARIES})
