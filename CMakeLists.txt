cmake_minimum_required(VERSION 2.8.3)
project(range_modules)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    sensor_msgs geometry_msgs roscpp tf2 tf2_ros
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

add_executable(filter_footprint src/filter_footprint.cpp)
add_dependencies(filter_footprint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_footprint ${catkin_LIBRARIES})

install(TARGETS range_to_laserscan
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS filter_footprint
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

