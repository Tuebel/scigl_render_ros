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
  image_transport
  tf2_ros
)
find_package(scigl_render REQUIRED)

# packaging
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES scigl_render_ros
 CATKIN_DEPENDS image_transport tf2_ros
 DEPENDS scigl_render
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# library
add_library(scigl_render_ros)
target_include_directories(scigl_render_ros PUBLIC
  ${catkin_INCLUDE_DIRS})
target_link_libraries(scigl_render_ros PUBLIC
  ${catkin_LIBRARIES}
  scigl_render)

# executables
add_executable(scigl_render_node)
target_link_libraries(scigl_render_node
  scigl_render_ros)

# installation
install(TARGETS scigl_render_ros scigl_render_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/scigl_render_ros/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)