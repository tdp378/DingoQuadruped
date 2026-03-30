cmake_minimum_required(VERSION 3.8)
project(dingo_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/joint_states_gazebo_plugin.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
