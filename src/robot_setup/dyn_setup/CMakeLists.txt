cmake_minimum_required(VERSION 3.16)
project(dyn_setup)
find_package(ament_cmake REQUIRED)
# `ros2 run dyn_setup tool_dyn_identify_wizard`: the load identification wizard (./scripts/setup.sh dyn runs it).
install(PROGRAMS scripts/tool_dyn_identify_wizard.py DESTINATION lib/${PROJECT_NAME} RENAME tool_dyn_identify_wizard)
ament_package()
