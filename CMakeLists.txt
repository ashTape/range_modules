cmake_minimum_required(VERSION 2.8.3)
project(range_modules)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

function(custom_library target)
  add_library(${target}_lib src/${target}.cpp)
  add_dependencies(${target}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${target}_lib ${catkin_LIBRARIES})  
endfunction()

function(custom_executable target)
  add_executable(${target}_node src/${target}_node.cpp)
  add_dependencies(${target}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${target}_lib)
  target_link_libraries(${target}_node ${catkin_LIBRARIES} ${target}_lib)
  install(TARGETS ${target}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endfunction()

file(GLOB MY_SOURCES "include/*.hpp")
foreach(source IN LISTS MY_SOURCES)
  get_filename_component(name ${source} NAME_WE) 
  custom_library(${name})
  custom_executable(${name})
endforeach()   

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})            
