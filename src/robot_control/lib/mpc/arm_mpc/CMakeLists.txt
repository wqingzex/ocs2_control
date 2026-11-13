cmake_minimum_required(VERSION 3.14)
project(arm_mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
# ocs2 first, robot_controller_core last: it pulls in pinocchio, which breaks ocs2_core's boost_log lookup if found earlier.
find_package(ocs2_mobile_manipulator REQUIRED)
find_package(mpc_core REQUIRED)
find_package(robot_controller_core REQUIRED)

add_library(${PROJECT_NAME} SHARED
        src/ArmMpcTask.cpp
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
        ament_index_cpp::ament_index_cpp
        ${ocs2_mobile_manipulator_TARGETS}
        ${mpc_core_TARGETS}
        ${robot_controller_core_TARGETS}
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

ament_export_dependencies(ament_index_cpp ocs2_mobile_manipulator mpc_core robot_controller_core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
