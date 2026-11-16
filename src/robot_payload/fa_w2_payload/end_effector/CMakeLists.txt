cmake_minimum_required(VERSION 3.16)
project(end_effector)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(end_effector)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  foreach(t test_hand test_hand_setup test_hand_step)
    ament_add_pytest_test(${t} test/${t}.py
      PYTHON_EXECUTABLE python3
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../common:${CMAKE_CURRENT_SOURCE_DIR}/../camera:${CMAKE_CURRENT_SOURCE_DIR}/../end_effector:${CMAKE_CURRENT_SOURCE_DIR}/../../../robot_utils/python)
  endforeach()
endif()
ament_package()
