cmake_minimum_required(VERSION 3.16)
project(modbus_hardware)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        gripper_hardware_common
)

find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()
find_package(geometry_msgs REQUIRED)
find_package(robot_comm_serial REQUIRED)
find_package(robot_comm_modbus REQUIRED)

## COMPILE
add_library(
        ${PROJECT_NAME}
        SHARED
        src/grippers/modbus_gripper_base.cpp
        src/grippers/changingtek_gripper.cpp
        src/grippers/jodell_gripper.cpp
        src/modbus_hardware.cpp
        src/hands/dexterous_hand_base.cpp
        src/hands/linkerhand/dexterous_hand_hardware.cpp
        src/hands/freedom/freedom_rs485_hardware.cpp
        src/hands/inspire/inspire_hand_hardware.cpp
        src/hands/theo/theo_hand_modbus_hardware.cpp
        src/hands/xhand1/xhand1_rs485_hardware.cpp
        src/sensors/kwr75_serial_client.cpp
        src/sensors/kwr75_force_torque_sensor.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${hardware_interface_TARGETS}
        ${pluginlib_TARGETS}
        ${rclcpp_TARGETS}
        ${rclcpp_lifecycle_TARGETS}
        ${geometry_msgs_TARGETS}
        gripper_hardware_common::gripper_hardware_common
        robot_comm_serial::robot_comm_serial
        robot_comm_modbus::robot_comm_modbus
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface modbus_hardware.xml)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_jodell_gripper test/test_jodell_gripper.cpp)
  target_link_libraries(test_jodell_gripper ${PROJECT_NAME})
endif()

## EXPORTS
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} robot_comm_serial robot_comm_modbus)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
