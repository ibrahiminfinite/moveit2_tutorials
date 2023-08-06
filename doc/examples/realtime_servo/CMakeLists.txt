add_executable(door_tutorial src/door_tutorial.cpp)
target_include_directories(door_tutorial PUBLIC include)
ament_target_dependencies(door_tutorial  ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS
  door_tutorial
  DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
