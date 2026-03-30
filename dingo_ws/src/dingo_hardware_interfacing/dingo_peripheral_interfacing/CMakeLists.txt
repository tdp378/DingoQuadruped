cmake_minimum_required(VERSION 3.8)
project(dingo_peripheral_interfacing)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/dingo_peripheral_interfacing)

install(PROGRAMS
  scripts/battery_voltage_checking.py
  scripts/dingo_lcd_interfacing.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
