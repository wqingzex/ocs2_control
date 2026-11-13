cmake_minimum_required(VERSION 3.14)
project(whole_body_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
# mpc_core first: it resolves the ocs2 packages before robot_controller_core pulls in pinocchio
# (the other order breaks ocs2_core's boost_log lookup).
find_package(mpc_core REQUIRED)
find_package(whole_body_mpc REQUIRED)
find_package(robot_controller_core REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME} SHARED src/WholeBodyController.cpp)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${OCS2_CXX_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${controller_interface_TARGETS}
    ${pluginlib_TARGETS}
    ${mpc_core_TARGETS}
    ${whole_body_mpc_TARGETS}
    ${robot_controller_core_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface whole_body_controller.xml)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_dependencies(controller_interface pluginlib mpc_core whole_body_mpc robot_controller_core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
