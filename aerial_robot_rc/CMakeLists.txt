cmake_minimum_required(VERSION 3.0.2)
project(aerial_robot_rc)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_msgs
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sbus2usb_parser
  CATKIN_DEPENDS aerial_robot_msgs roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(sbus2usb_parser
  src/sbus2usb_parser.cpp
)
add_dependencies(sbus2usb_parser ${catkin_EXPORTED_TARGETS})
target_link_libraries(sbus2usb_parser ${catkin_LIBRARIES})

add_executable(sbus2usb_rc_node
  src/sbus2usb_rc_node.cpp
)
add_dependencies(sbus2usb_rc_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sbus2usb_rc_node ${catkin_LIBRARIES} sbus2usb_parser)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS sbus2usb_parser sbus2usb_rc_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/rc_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_sbus2usb_parser test/test_sbus2usb_parser.cpp)
  if(TARGET test_sbus2usb_parser)
    target_link_libraries(test_sbus2usb_parser sbus2usb_parser ${catkin_LIBRARIES})
  endif()
endif()
