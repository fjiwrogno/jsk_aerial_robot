cmake_minimum_required(VERSION 3.0.2)
project(flamingo)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  spinal
  pluginlib
  roscpp
  std_msgs
  )

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs pluginlib roscpp spinal std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  )

add_library(flamingo_controller src/control/flamingo_controller.cpp)
target_link_libraries(flamingo_controller ${catkin_LIBRARIES})

add_library(flamingo_robot_model src/model/flamingo_robot_model.cpp)
target_link_libraries(flamingo_robot_model ${catkin_LIBRARIES})

add_library(flamingo_navigation src/flamingo_navigation.cpp)
target_link_libraries(flamingo_navigation ${catkin_LIBRARIES})

add_executable(flamingo_param_manager src/flamingo_param_manager.cpp)
target_link_libraries(flamingo_param_manager ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

install(DIRECTORY config plugins launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
