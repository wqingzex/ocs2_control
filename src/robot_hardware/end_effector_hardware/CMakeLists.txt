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
)

find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()
# Transport only; the device protocols live in this package.
find_package(robot_comm_core REQUIRED)
find_package(robot_comm_serial REQUIRED)
find_package(robot_comm_modbus REQUIRED)
find_package(robot_comm_can REQUIRED)

# Gripper / hand Modbus protocol (register tables, unit conversion, command building): header-only, depends on rclcpp
# only. Linked by the plugin library (the bus devices).
add_library(gripper_protocol INTERFACE)
target_include_directories(gripper_protocol INTERFACE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(gripper_protocol INTERFACE ${rclcpp_TARGETS})

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
        # LinkerHand on classic CAN
        src/hands/linkerhand/linkerhand_can_hardware.cpp
        # end effectors on a robot_comm bus (USB or an arm's flange port)
        src/bus/bus_device.cpp
        src/bus/jodell_rg75.cpp
        src/bus/linkerhand_l6.cpp
        src/bus/xhand1.cpp
        src/bus/end_effector_hardware.cpp
        src/bus/bus_probe.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
        hardware_interface pluginlib rclcpp rclcpp_lifecycle rcl_interfaces std_msgs geometry_msgs)
target_link_libraries(${PROJECT_NAME} PUBLIC
        gripper_protocol
        robot_comm_core::robot_comm_bus
        robot_comm_serial::robot_comm_serial
        robot_comm_modbus::robot_comm_modbus
        robot_comm_can::robot_comm_can
        end_effector_can_protocols
)

pluginlib_export_plugin_description_file(hardware_interface end_effector_hardware.xml)

# Bus measurement through the end effector that uses the bus (started by `start.sh bus_probe`).
add_executable(bus_probe tools/bus_probe.cpp)
target_link_libraries(bus_probe ${PROJECT_NAME})
install(TARGETS bus_probe RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
        TARGETS ${PROJECT_NAME} end_effector_can_protocols gripper_protocol
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Bus devices and EndEffectorHardware on a MockBus: initialization, command / status frames, failures.
  ament_add_gtest(test_bus_devices test/test_bus_devices.cpp)
  target_link_libraries(test_bus_devices ${PROJECT_NAME})
  ament_add_gtest(test_end_effector_hardware test/test_end_effector_hardware.cpp)
  target_link_libraries(test_end_effector_hardware ${PROJECT_NAME})
  ament_target_dependencies(test_end_effector_hardware hardware_interface rclcpp rclcpp_lifecycle)

  # The bus measurement on a MockBus with simulated devices: round trips, loss, gaps, failed devices.
  ament_add_gtest(test_bus_probe test/test_bus_probe.cpp)
  target_link_libraries(test_bus_probe ${PROJECT_NAME})

  # The LinkerHand CAN protocol: frames in and out, without ROS and without a bus.
  ament_add_gtest(test_linkerhand_can_protocol test/test_linkerhand_can_protocol.cpp)
  target_link_libraries(test_linkerhand_can_protocol end_effector_can_protocols)

  # The LinkerHand CAN driver on a MockCanBus: init checks, feedback, commands, tool limits, bus errors.
  ament_add_gtest(test_linkerhand_can_hardware test/test_linkerhand_can_hardware.cpp)
  target_link_libraries(test_linkerhand_can_hardware ${PROJECT_NAME} robot_comm_can::robot_comm_can)
  ament_target_dependencies(test_linkerhand_can_hardware hardware_interface rclcpp rclcpp_lifecycle)
endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} robot_comm_core robot_comm_serial robot_comm_modbus robot_comm_can)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
