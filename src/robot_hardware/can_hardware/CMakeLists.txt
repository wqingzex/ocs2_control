cmake_minimum_required(VERSION 3.16)
project(can_hardware)

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
)

find_package(ament_cmake REQUIRED)
find_package(robot_comm_can REQUIRED)
foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()

# Device protocols: plain functions on robot_comm::CanFrame, no ROS (also linked by the protocol tests).
add_library(can_hardware_protocols STATIC
        src/hands/linkerhand/linkerhand_can_protocol.cpp
)
target_compile_features(can_hardware_protocols PUBLIC cxx_std_17)
target_include_directories(can_hardware_protocols PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(can_hardware_protocols PUBLIC robot_comm_can::robot_comm_can)
set_target_properties(can_hardware_protocols PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME} SHARED
        src/hands/linkerhand/linkerhand_can_hardware.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME} PUBLIC can_hardware_protocols robot_comm_can::robot_comm_can)

pluginlib_export_plugin_description_file(hardware_interface can_hardware.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
        TARGETS ${PROJECT_NAME} can_hardware_protocols
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} robot_comm_can)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # The LinkerHand protocol: frames in and out, without ROS and without a bus.
  ament_add_gtest(test_linkerhand_can_protocol test/test_linkerhand_can_protocol.cpp)
  target_link_libraries(test_linkerhand_can_protocol can_hardware_protocols)

  # The driver on a MockCanBus: init checks, feedback, commands, tool limits, bus errors.
  ament_add_gtest(test_linkerhand_can_hardware test/test_linkerhand_can_hardware.cpp)
  target_link_libraries(test_linkerhand_can_hardware ${PROJECT_NAME} robot_comm_can::robot_comm_can)
  ament_target_dependencies(test_linkerhand_can_hardware ${THIS_PACKAGE_INCLUDE_DEPENDS})
endif()

ament_package()
