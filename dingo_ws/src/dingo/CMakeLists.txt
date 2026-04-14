cmake_minimum_required(VERSION 3.8)
project(dingo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/dingo)

install(PROGRAMS
  scripts/dingo_driver.py
  scripts/run_robot.py
  scripts/cmd_vel_to_joy_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
