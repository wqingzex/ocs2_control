cmake_minimum_required(VERSION 3.16)
project(robot_app LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Double-click entry in the workspace root, built natively on both x86_64 and ARM64.
get_filename_component(ROBOT_APP_WS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
add_executable(console_launcher src/console_launcher.cpp)
set_target_properties(console_launcher PROPERTIES
  OUTPUT_NAME "robot_control_suite控制台"
  RUNTIME_OUTPUT_DIRECTORY "${ROBOT_APP_WS_ROOT}")
# Use an ELF executable rather than a PIE shared-object MIME type in desktop file managers.
target_link_options(console_launcher PRIVATE -no-pie)

# Module supervision, command line, web console, recording and diagnostics (start.sh, robot_control_suite控制台)
ament_python_install_package(robot_app)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  foreach(t test_launch_config test_units test_stop_hook test_console test_home_setup)
    ament_add_pytest_test(${t} test/${t}.py
      PYTHON_EXECUTABLE python3
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../../robot_teleop/xr_teleop:${CMAKE_CURRENT_SOURCE_DIR}/../../robot_teleop/joystick_teleop:${CMAKE_CURRENT_SOURCE_DIR}/../../robot_setup/robot_config:${CMAKE_CURRENT_SOURCE_DIR}/../../robot_setup/camera_setup:${CMAKE_CURRENT_SOURCE_DIR}/../../robot_setup/end_effector_setup:${CMAKE_CURRENT_SOURCE_DIR}/../../robot_utils/python:${CMAKE_CURRENT_SOURCE_DIR}/../../robot_launch/launch_helpers)
  endforeach()
  ament_add_pytest_test(test_console_launcher test/test_console_launcher.py
    PYTHON_EXECUTABLE python3
    ENV CONSOLE_LAUNCHER=$<TARGET_FILE:console_launcher>)
endif()

ament_package()
