cmake_minimum_required(VERSION 3.16)
project(xr_setup)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(xr_setup)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  foreach(t test_xr_session)
    ament_add_pytest_test(${t} test/${t}.py
      PYTHON_EXECUTABLE python3
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../robot_config:${CMAKE_CURRENT_SOURCE_DIR}/../../robot_teleop/xr_teleop:${CMAKE_CURRENT_SOURCE_DIR}/../../robot_launch/launch_helpers:${CMAKE_CURRENT_SOURCE_DIR}/../../robot_utils/python)
  endforeach()
endif()
ament_package()
