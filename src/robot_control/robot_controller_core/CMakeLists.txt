cmake_minimum_required(VERSION 3.8)
project(robot_controller_core)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        hardware_interface
        controller_interface
        std_msgs
        rclcpp
        rclcpp_action
        pinocchio
        urdf
        ament_index_cpp
        tf2_geometry_msgs
        tf2
        geometry_msgs
        trajectory_msgs
        nav_msgs
        tf2_ros
        robot_control_msgs
        robot_dynamics
        trajectory_manager
        robot_utils
)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

foreach (Dependency IN ITEMS ${dependencies})
    find_package(${Dependency} REQUIRED)
endforeach ()


set(CORE_SOURCES
        src/FsmControllerBase.cpp
        src/WaistCommands.cpp
        src/StateHome.cpp
        src/FsmController.cpp
        src/StateHold.cpp
        src/StateMoveJ.cpp
)

add_library(${PROJECT_NAME} SHARED
        ${CORE_SOURCES}
)

# Pinocchio headers (ROS Jazzy) can trigger -Wreorder in template instantiation paths.
# Suppress it only for this target to keep workspace-wide warnings strict.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE -Wno-reorder)
endif()

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE
        src
)

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${hardware_interface_TARGETS}
        ${controller_interface_TARGETS}
        ${std_msgs_TARGETS}
        ${rclcpp_TARGETS}
        ${robot_utils_TARGETS}
        pinocchio::pinocchio
        ${urdf_TARGETS}
        ament_index_cpp::ament_index_cpp
        ${tf2_geometry_msgs_TARGETS}
        ${tf2_TARGETS}
        ${tf2_ros_TARGETS}
        ${geometry_msgs_TARGETS}
        ${trajectory_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${robot_control_msgs_TARGETS}
        robot_dynamics::robot_dynamics
        trajectory_manager::trajectory_manager
        Eigen3::Eigen
)


target_link_libraries(${PROJECT_NAME} PRIVATE
        pthread
        rt
)


install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
