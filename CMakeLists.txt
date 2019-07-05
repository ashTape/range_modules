cmake_minimum_required(VERSION 2.8.3)
project(range_modules)

add_compile_options(-std=c++11)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf2 tf2_ros
)

#########################################################################rectories(
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(range_to_laserscan src/range_to_laserscan.cpp)
add_dependencies(range_to_laserscan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(range_to_laserscan ${catkin_LIBRARIES})

add_executable(range_filter_footprint src/range_filter_footprint.cpp)
add_dependencies(range_filter_footprint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(range_filter_footprint ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS range_to_laserscan
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS range_filter_footprint
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

################################################################################
# Test
################################################################################
