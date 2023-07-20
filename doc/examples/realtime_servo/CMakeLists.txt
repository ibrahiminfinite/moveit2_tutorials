add_executable(servo_door_opening_demo src/servo_door_opening_demo.cpp)
target_include_directories(servo_door_opening_demo PUBLIC include)
ament_target_dependencies(servo_door_opening_demo  ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS
    servo_door_opening_demo
  DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
