cmake_minimum_required(VERSION 3.14)
project(whole_body_mpc)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_control_msgs REQUIRED)
# ocs2 first, robot_controller_core last: it pulls in pinocchio, which breaks ocs2_core's boost_log lookup if found earlier.
find_package(ocs2_mobile_manipulator REQUIRED)
find_package(mpc_core REQUIRED)
find_package(robot_controller_core REQUIRED)

add_library(${PROJECT_NAME}
    src/mode/HumanoidModeSchedule.cpp
    src/mode/SwitchedHumanoidReferenceManager.cpp
    src/cost/WaistPhiPositiveBiasCost.cpp
    src/constraint/BimanualCouplingConstraint.cpp
    src/constraint/BodyTrackingConstraint.cpp
    src/constraint/HeadTrackingConstraint.cpp
    src/constraint/WaistJointLockConstraint.cpp
    src/constraint/HeadCouplingConstraint.cpp
    src/constraint/HeadDefaultLockConstraint.cpp
    src/constraint/ArmJointLockConstraint.cpp
    src/constraint/CustomJointLockConstraint.cpp
    src/constraint/HeadMidpointGazeConstraint.cpp
    src/constraint/HomeJointReferenceConstraint.cpp
    src/constraint/WholeBodyEndEffectorConstraint.cpp
    src/WholeBodyInterface.cpp
    src/WholeBodyMpcTask.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${OCS2_CXX_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${ocs2_mobile_manipulator_TARGETS}
    ${mpc_core_TARGETS}
    ${robot_controller_core_TARGETS}
    ${robot_control_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${tf2_TARGETS}
    ${tf2_msgs_TARGETS}
    ${tf2_ros_TARGETS}
    ${rclcpp_lifecycle_TARGETS}
    ament_index_cpp::ament_index_cpp
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

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(ament_index_cpp geometry_msgs rclcpp_lifecycle std_msgs tf2 tf2_msgs tf2_ros robot_control_msgs
    ocs2_mobile_manipulator mpc_core robot_controller_core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_base_velocity_reference test/test_base_velocity_reference.cpp)
  target_link_libraries(test_base_velocity_reference ${PROJECT_NAME})
endif()

ament_package()
