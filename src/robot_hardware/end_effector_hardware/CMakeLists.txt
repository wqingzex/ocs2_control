cmake_minimum_required(VERSION 3.16)
project(end_effector_hardware)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        rcl_interfaces
        std_msgs
        geometry_msgs
        gripper_hardware_common
)

find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()
# Transport only; the device protocols live in this package.
find_package(robot_comm_serial REQUIRED)
find_package(robot_comm_modbus REQUIRED)
find_package(robot_comm_can REQUIRED)

# CAN device protocols: plain functions on robot_comm::CanFrame, no ROS (also linked by the protocol tests).
add_library(end_effector_can_protocols STATIC
        src/hands/linkerhand/linkerhand_can_protocol.cpp
)
target_compile_features(end_effector_can_protocols PUBLIC cxx_std_17)
target_include_directories(end_effector_can_protocols PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(end_effector_can_protocols PUBLIC robot_comm_can::robot_comm_can)
set_target_properties(end_effector_can_protocols PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME} SHARED
        # grippers (RS485 / Modbus)
        src/grippers/modbus_gripper_base.cpp
        src/grippers/changingtek_gripper.cpp
        src/grippers/jodell_gripper.cpp
        src/grippers/gripper_modbus_hardware.cpp
        # dexterous hands
        src/hands/dexterous_hand_base.cpp
        src/hands/linkerhand/linkerhand_rs485_hardware.cpp
        src/hands/linkerhand/linkerhand_can_hardware.cpp
        src/hands/freedom/freedom_rs485_hardware.cpp
        src/hands/inspire/inspire_rs485_hardware.cpp
        src/hands/theo/theo_hand_rs485_hardware.cpp
        src/hands/xhand1/xhand1_rs485_hardware.cpp
        # sensors
        src/sensors/kwr75/kwr75_serial_client.cpp
        src/sensors/kwr75/kwr75_force_torque_sensor.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
        hardware_interface pluginlib rclcpp rclcpp_lifecycle rcl_interfaces std_msgs geometry_msgs)
target_link_libraries(${PROJECT_NAME} PUBLIC
        gripper_hardware_common::gripper_hardware_common
        robot_comm_serial::robot_comm_serial
        robot_comm_modbus::robot_comm_modbus
        robot_comm_can::robot_comm_can
        end_effector_can_protocols
)

pluginlib_export_plugin_description_file(hardware_interface end_effector_hardware.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
        TARGETS ${PROJECT_NAME} end_effector_can_protocols
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_jodell_gripper test/test_jodell_gripper.cpp)
  target_link_libraries(test_jodell_gripper ${PROJECT_NAME})

  # The LinkerHand CAN protocol: frames in and out, without ROS and without a bus.
  ament_add_gtest(test_linkerhand_can_protocol test/test_linkerhand_can_protocol.cpp)
  target_link_libraries(test_linkerhand_can_protocol end_effector_can_protocols)

  # The LinkerHand CAN driver on a MockCanBus: init checks, feedback, commands, tool limits, bus errors.
  ament_add_gtest(test_linkerhand_can_hardware test/test_linkerhand_can_hardware.cpp)
  target_link_libraries(test_linkerhand_can_hardware ${PROJECT_NAME} robot_comm_can::robot_comm_can)
  ament_target_dependencies(test_linkerhand_can_hardware hardware_interface rclcpp rclcpp_lifecycle)
endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} robot_comm_serial robot_comm_modbus robot_comm_can)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
