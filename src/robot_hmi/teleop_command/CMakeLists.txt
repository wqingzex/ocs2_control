cmake_minimum_required(VERSION 3.14)
project(teleop_command)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(TARGET_MANAGER_INCLUDE_DEPENDS
        rclcpp
        geometry_msgs
        visualization_msgs
        interactive_markers
        robot_control_msgs
        robot_controller_core
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        nav_msgs
        robot_log
        Eigen3
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${TARGET_MANAGER_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()

add_library(${PROJECT_NAME} SHARED
        src/TeleopCommand.cpp
        src/MarkerFactory.cpp
        src/ControlInputHandler.cpp
        src/VRInputHandler.cpp
        src/marker/HeadMarker.cpp
        src/marker/ArmMarker.cpp
        src/marker/BodyMarker.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE
        src)

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${rclcpp_TARGETS}
        ${robot_log_TARGETS}
        ${geometry_msgs_TARGETS}
        ${visualization_msgs_TARGETS}
        ${interactive_markers_TARGETS}
        ${robot_control_msgs_TARGETS}
        robot_controller_core::robot_controller_core
        ${tf2_ros_TARGETS}
        ${tf2_geometry_msgs_TARGETS}
        ${std_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        Eigen3::Eigen
)

add_executable(teleop_command_node src/teleop_command_node.cpp)
target_link_libraries(teleop_command_node ${PROJECT_NAME})

# Ensure the loader finds libteleop_command.so next to the executable (same dir as install).
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set_target_properties(teleop_command_node PROPERTIES
    BUILD_RPATH "$ORIGIN"
    INSTALL_RPATH "$ORIGIN"
    BUILD_WITH_INSTALL_RPATH FALSE
  )
endif()

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin
)

install(
        TARGETS teleop_command_node
        DESTINATION lib/${PROJECT_NAME}
)

# Launch files are now handled by launch_helpers utilities
# No need to install launch directory anymore

install(
        DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_vr_alignment test/test_vr_alignment.cpp)
  target_include_directories(test_vr_alignment PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_vr_alignment Eigen3::Eigen)
endif()

ament_export_dependencies(${TARGET_MANAGER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
