add_rostest(flight_control.test ARGS headless:=true airframe:=quad)

add_rostest_gtest(nami_transformation_stability_test
  transformation_stability.test
  transformation_stability.cpp
  )
target_link_libraries(nami_transformation_stability_test
  nami_robot_model
  ${catkin_LIBRARIES}
  )
