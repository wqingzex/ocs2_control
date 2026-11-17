cmake_minimum_required(VERSION 3.8)
project(joystick_teleop)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_control_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(joystick_teleop src/joystick_teleop.cpp)
target_include_directories(joystick_teleop
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(joystick_teleop PUBLIC
        ${rclcpp_TARGETS}
        ${sensor_msgs_TARGETS}
        ${std_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${robot_control_msgs_TARGETS}
        ${control_msgs_TARGETS}
        ${rclcpp_action_TARGETS}
)

ament_python_install_package(joystick_teleop)
install(TARGETS joystick_teleop DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
