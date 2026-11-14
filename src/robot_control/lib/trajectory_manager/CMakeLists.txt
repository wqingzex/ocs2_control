cmake_minimum_required(VERSION 3.8)
project(trajectory_manager)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)

set(dependencies
        rclcpp
        geometry_msgs
        robot_control_msgs
        robot_dynamics
        motion_planning
)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

foreach (Dependency IN ITEMS ${dependencies})
    find_package(${Dependency} REQUIRED)
endforeach ()

add_library(${PROJECT_NAME} SHARED
        src/JointTrajectoryManager.cpp
        src/CartesianTrajectoryManager.cpp
        src/JointSpeedStopPlanner.cpp
        src/WaistLiftingPlanner.cpp
        src/TrajectoryRecorder.cpp
)

# Pinocchio headers (ROS Jazzy) can trigger -Wreorder in template instantiation paths.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE -Wno-reorder)
endif()

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${rclcpp_TARGETS}
        ${geometry_msgs_TARGETS}
        ${robot_control_msgs_TARGETS}
        robot_dynamics::robot_dynamics
        motion_planning::motion_planning
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
