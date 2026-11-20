cmake_minimum_required(VERSION 3.16)
project(robot_config)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(robot_config)
# `ros2 run robot_config tool_dyn_identify_wizard`: the load identification wizard (it uses the Marvin SDK of marvin_hardware).
install(PROGRAMS scripts/tool_dyn_identify_wizard.py DESTINATION lib/${PROJECT_NAME} RENAME tool_dyn_identify_wizard)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  foreach(t test_device_setup test_load_identify test_setup_script test_config_sections
          test_config_edit test_boundaries test_template test_mappings test_sync test_setup_steps_cli)
    ament_add_pytest_test(${t} test/${t}.py
      PYTHON_EXECUTABLE python3
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../robot_config:${CMAKE_CURRENT_SOURCE_DIR}/../camera_setup:${CMAKE_CURRENT_SOURCE_DIR}/../end_effector_setup:${CMAKE_CURRENT_SOURCE_DIR}/../../robot_utils/python)
  endforeach()
endif()
ament_package()
