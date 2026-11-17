cmake_minimum_required(VERSION 3.16)
project(dexcap_teleop)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ruckig REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# --- DexCap SDK (prebuilt, src/third_party/dexcap-sdk-cpp) ------------------------------------------------------------
set(DEXCAP_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/dexcap-sdk-cpp" CACHE PATH "DexCap SDK directory")
set(DEXCAP_SDK_LIB_DIR "${DEXCAP_SDK_ROOT}/libs/linux")
if(NOT EXISTS "${DEXCAP_SDK_LIB_DIR}/libDexCap.so")
  message(FATAL_ERROR "DexCap SDK not found at ${DEXCAP_SDK_ROOT} (set -DDEXCAP_SDK_ROOT=)")
endif()
if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  message(WARNING "The DexCap SDK in third_party is built for x86_64; the device nodes will not link on ${CMAKE_SYSTEM_PROCESSOR}")
endif()

add_library(dexcap_device STATIC src/dexcap_device.cpp)
set_target_properties(dexcap_device PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(dexcap_device PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "${DEXCAP_SDK_ROOT}/include"
  "${DEXCAP_SDK_ROOT}/include/cpp")
target_link_libraries(dexcap_device
  "${DEXCAP_SDK_LIB_DIR}/libserial.so"
  "${DEXCAP_SDK_LIB_DIR}/libyaml-cpp.so"
  "${DEXCAP_SDK_LIB_DIR}/libDexCap.so")
# Not linked: the SDK's libusb-1.0.so. Nothing here or in libDexCap uses it, and its SONAME libusb-1.0.so.0 has no
# matching file in the SDK, so with --no-as-needed the nodes would fail to start where libusb-1.0-0 is not installed.

# Device nodes: raw joint states of the exoskeleton (waist, arms) and the gloves.
add_executable(dexcap_upper_body_node src/dexcap_upper_body_node.cpp)
add_executable(dexcap_glove_node src/dexcap_glove_node.cpp)
foreach(target dexcap_upper_body_node dexcap_glove_node)
  target_link_libraries(${target} dexcap_device)
  # libDexCap depends on the SDK's own yaml / serial / udev: DT_RPATH (not DT_RUNPATH) so that directory is also
  # searched for those indirect dependencies.
  target_link_options(${target} PRIVATE "-Wl,--disable-new-dtags" "-Wl,--no-as-needed")
  ament_target_dependencies(${target} rclcpp sensor_msgs)
  set_target_properties(${target} PROPERTIES BUILD_RPATH "${DEXCAP_SDK_LIB_DIR}" INSTALL_RPATH "$ORIGIN/sdk")
endforeach()

# Mappers: glove -> gripper / hand targets, exoskeleton arms -> dual-arm joint targets.
add_executable(dexcap_gripper_mapper src/dexcap_gripper_mapper.cpp)
ament_target_dependencies(dexcap_gripper_mapper kdl_parser rclcpp sensor_msgs std_msgs)
target_link_libraries(dexcap_gripper_mapper ${kdl_parser_TARGETS})

add_executable(dexcap_hand_mapper src/dexcap_hand_mapper.cpp)
ament_target_dependencies(dexcap_hand_mapper rclcpp sensor_msgs std_msgs std_srvs)

add_library(trajectory_filter STATIC src/trajectory_filter.cpp)
target_include_directories(trajectory_filter PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(trajectory_filter PUBLIC ruckig::ruckig)
add_executable(arm_joint_mapper src/arm_joint_mapper_node.cpp)
target_link_libraries(arm_joint_mapper trajectory_filter)
ament_target_dependencies(arm_joint_mapper rclcpp sensor_msgs std_msgs)

install(TARGETS dexcap_upper_body_node dexcap_glove_node dexcap_gripper_mapper dexcap_hand_mapper arm_joint_mapper
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY "${DEXCAP_SDK_LIB_DIR}/"
  DESTINATION lib/${PROJECT_NAME}/sdk
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.so*")
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

ament_package()
