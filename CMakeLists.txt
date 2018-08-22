cmake_minimum_required(VERSION 2.8.3)
project(scigl_render_ros)

# Default ROS Kinetic configurations for compilation
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  tf2_ros)
find_package(OpenCV REQUIRED)
find_package(scigl_render REQUIRED)

# packaging
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES scigl_render_ros
 CATKIN_DEPENDS cv_bridge image_transport roscpp tf2_ros
 DEPENDS OpenCV scigl_render)


# library
add_library(scigl_render_ros)
target_sources(scigl_render_ros PRIVATE
  src/ar_render.cpp)
target_include_directories(scigl_render_ros PUBLIC
  ${catkin_INCLUDE_DIRS}
  include)
target_link_libraries(scigl_render_ros PUBLIC
  ${catkin_LIBRARIES}
  scigl_render)

# executables
add_executable(ar_render_node)
target_sources(ar_render_node PRIVATE 
  src/ar_render_node.cpp
  src/ar_render_main.cpp)
target_link_libraries(ar_render_node
  ${OpenCV_LIBS}
  scigl_render_ros)

# installation
install(TARGETS scigl_render_ros ar_render_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/scigl_render_ros/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)