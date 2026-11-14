cmake_minimum_required(VERSION 3.14)
project(end_effector_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(robot_controller_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED src/EndEffectorController.cpp src/HandEndEffector.cpp src/GripperEndEffector.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE src)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${controller_interface_TARGETS}
    ${pluginlib_TARGETS}
    ${robot_controller_core_TARGETS}
    ${std_msgs_TARGETS}
    ${tf2_ros_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface end_effector_controller.xml)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_dependencies(controller_interface pluginlib robot_controller_core std_msgs tf2_ros)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
