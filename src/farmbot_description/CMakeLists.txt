cmake_minimum_required(VERSION 3.8)
project(farmbot_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY launch assets config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES farmbot_genesis.urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    src/teleop_keyboard.py
    src/joint_state_publisher_node.py
    src/motion_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
