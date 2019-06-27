cmake_minimum_required(VERSION 2.8.3)
project(range_to_laserscan)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    sensor_msgs roscpp
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(range_to_laserscan src/range_to_laserscan.cpp)
add_dependencies(range_to_laserscan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(range_to_laserscan ${catkin_LIBRARIES})

install(TARGETS range_to_laserscan
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

