cmake_minimum_required(VERSION 3.0.2)
project(nami)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  spinal
  pluginlib
  roscpp
  )

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs pluginlib roscpp spinal
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(nami_controller
  src/control/nami_controller.cpp
  src/control/nami_lqi_controller.cpp
  )
target_link_libraries(nami_controller ${catkin_LIBRARIES})

add_library(nami_robot_model src/model/nami_robot_model.cpp)
target_link_libraries(nami_robot_model ${catkin_LIBRARIES})

add_library(nami_navigation src/nami_navigation.cpp)
target_link_libraries(nami_navigation ${catkin_LIBRARIES})

install(DIRECTORY config doc plugins launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

catkin_install_python(PROGRAMS
  scripts/transformation_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
