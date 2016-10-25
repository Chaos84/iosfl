cmake_minimum_required(VERSION 2.8.3)
project(iosfl)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
)

catkin_package(INCLUDE_DIRS include LIBRARIES iosfl CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs tf DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(iosfl src/iosfl.cpp)
target_link_libraries(iosfl ${catkin_LIBRARIES})

add_executable(iosfl_node src/iosfl_node.cpp)
target_link_libraries(iosfl_node ${catkin_LIBRARIES} iosfl)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)