add_rostest(flight_control.test ARGS headless:=true airframe:=quad)

catkin_add_gtest(test_rc_thrust_mapper
  test_rc_thrust_mapper.cpp
  ../src/control/rc_thrust_mapper.cpp
  )
if(TARGET test_rc_thrust_mapper)
  target_link_libraries(test_rc_thrust_mapper ${catkin_LIBRARIES})
endif()

add_rostest_gtest(test_rc_mapper
  rc_mapper.test
  test_rc_mapper.cpp
  ../src/control/rc_mapper.cpp
  )
if(TARGET test_rc_mapper)
  target_link_libraries(test_rc_mapper ${catkin_LIBRARIES})
endif()
