cmake_minimum_required(VERSION 3.14)
project(whole_body_mpc_nodes)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(ocs2_self_collision_visualization REQUIRED)
find_package(ocs2_mobile_manipulator REQUIRED)
find_package(whole_body_mpc REQUIRED)
find_package(ocs2_mobile_manipulator_ros REQUIRED)

add_library(${PROJECT_NAME}
    src/WholeBodyMpcNode.cpp
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
    ${tf2_ros_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
    ${urdf_TARGETS}
    ${kdl_parser_TARGETS}
    ${robot_state_publisher_TARGETS}
    ${sensor_msgs_TARGETS}
    ${interactive_markers_TARGETS}
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${ocs2_ros_interfaces_TARGETS}
    ${ocs2_pinocchio_interface_TARGETS}
    ${ocs2_self_collision_TARGETS}
    ${ocs2_self_collision_visualization_TARGETS}
    ${ocs2_mobile_manipulator_TARGETS}
    ${whole_body_mpc_TARGETS}
    ${ocs2_mobile_manipulator_ros_TARGETS}
)

add_executable(whole_body_mpc src/WholeBodyMpcNode.cpp)
target_link_libraries(whole_body_mpc ${PROJECT_NAME})

add_executable(whole_body_dummy src/WholeBodyDummyNode.cpp)
target_link_libraries(whole_body_dummy ${PROJECT_NAME})

add_executable(whole_body_target src/WholeBodyTargetNode.cpp)
target_link_libraries(whole_body_target ${PROJECT_NAME})

install(DIRECTORY include DESTINATION include/${PROJECT_NAME})
install(
    TARGETS
        whole_body_mpc
        whole_body_dummy
        whole_body_target
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
