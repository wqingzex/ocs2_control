cmake_minimum_required(VERSION 3.16)
project(camera_setup)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(camera_setup)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  foreach(t test_camera_discovery test_camera_session test_camera_step)
    ament_add_pytest_test(${t} test/${t}.py
      PYTHON_EXECUTABLE python3
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../robot_config:${CMAKE_CURRENT_SOURCE_DIR}/../camera_setup:${CMAKE_CURRENT_SOURCE_DIR}/../end_effector_setup:${CMAKE_CURRENT_SOURCE_DIR}/../../robot_utils/python:${CMAKE_CURRENT_SOURCE_DIR}/../../robot_launch/launch_helpers)
  endforeach()
endif()
ament_package()
