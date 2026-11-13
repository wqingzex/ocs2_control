cmake_minimum_required(VERSION 3.16)
project(juxie_hardware)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# 生成IDE索引符号
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 依赖包
set(THIS_PACKAGE_INCLUDE_DEPENDS
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
find_package(robot_comm_can REQUIRED)
find_package(robot_log REQUIRED)
foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()

# 编译核心库
add_library(
        ${PROJECT_NAME}
        SHARED
        src/jx_hardware.cpp
        src/juxie_protocol.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# 链接依赖
ament_target_dependencies(
        ${PROJECT_NAME} PUBLIC
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC robot_comm_can::robot_comm_can ${robot_log_TARGETS})

# 电机模拟器（Juxie 协议的纯逻辑），只给单元测试用
add_library(juxie_motor_sim STATIC src/juxie_motor_sim.cpp)
target_include_directories(juxie_motor_sim PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_link_libraries(juxie_motor_sim PUBLIC robot_comm_can::robot_comm_can)
target_sources(juxie_motor_sim PRIVATE src/juxie_protocol.cpp)
target_compile_features(juxie_motor_sim PUBLIC cxx_std_17)
set_target_properties(juxie_motor_sim PROPERTIES POSITION_INDEPENDENT_CODE ON)


# 导出硬件插件
pluginlib_export_plugin_description_file(hardware_interface juxie_hardware.xml)

# 安装头文件
install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

# 安装库文件
install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# 导出目标
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} robot_comm_can)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # The protocol is plain functions on robot_comm::CanFrame: tested without ROS and without hardware.
  ament_add_gtest(test_juxie_protocol test/test_juxie_protocol.cpp src/juxie_protocol.cpp)
  target_include_directories(test_juxie_protocol PRIVATE include)
  target_link_libraries(test_juxie_protocol robot_comm_can::robot_comm_can)

  # The driver itself on a MockCanBus: activation, command ramp, feedback, faults.
  ament_add_gtest(test_jx_hardware test/test_jx_hardware.cpp)
  target_link_libraries(test_jx_hardware ${PROJECT_NAME} robot_comm_can::robot_comm_can)
  ament_target_dependencies(test_jx_hardware ${THIS_PACKAGE_INCLUDE_DEPENDS})

  # The driver and a model of the motors in a closed loop, over the mock bus.
  ament_add_gtest(test_jx_closed_loop test/test_jx_closed_loop.cpp)
  target_link_libraries(test_jx_closed_loop ${PROJECT_NAME} juxie_motor_sim robot_comm_can::robot_comm_can)
  ament_target_dependencies(test_jx_closed_loop ${THIS_PACKAGE_INCLUDE_DEPENDS})
endif()

ament_package()