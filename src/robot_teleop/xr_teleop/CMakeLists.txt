cmake_minimum_required(VERSION 3.16)
project(xr_teleop LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_utils REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

# ---------------------------------------------------------------------------
# Core logic (no SDK, no ROS): JSON parsing, pose conversion, button events.
# ---------------------------------------------------------------------------
add_library(xr_teleop_core STATIC
  src/xr_state_json.cpp
  src/xr_pose_converter.cpp
  src/button_event_encoder.cpp)
target_include_directories(xr_teleop_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(xr_teleop_core PUBLIC Eigen3::Eigen PRIVATE nlohmann_json::nlohmann_json)
set_target_properties(xr_teleop_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# XRoboToolkit PC Service SDK (installed by the PC Service deb); override with -DXRT_SDK_DIR=<dir>
# or env XRT_SDK_DIR (dir containing include/ and x64|aarch64/).
set(XRT_SDK_DIR "$ENV{XRT_SDK_DIR}" CACHE PATH "XRoboToolkit PXREARobotSDK root")
set(_xrt_hints ${XRT_SDK_DIR} /opt/apps/roboticsservice/SDK)
find_path(PXREA_INCLUDE_DIR PXREARobotSDK.h HINTS ${_xrt_hints} PATH_SUFFIXES include NO_DEFAULT_PATH)
find_library(PXREA_LIBRARY PXREARobotSDK HINTS ${_xrt_hints}
  PATH_SUFFIXES x64 aarch64 arm64 linux/64 linux_aarch64/64 lib NO_DEFAULT_PATH)

if(PXREA_INCLUDE_DIR AND PXREA_LIBRARY)
  message(STATUS "xr_teleop: XRoboToolkit SDK found: ${PXREA_LIBRARY}")
  get_filename_component(PXREA_LIBRARY_DIR "${PXREA_LIBRARY}" DIRECTORY)

  add_library(xr_bridge SHARED src/xr_sdk_client.cpp src/xr_bridge_node.cpp)
  target_include_directories(xr_bridge PRIVATE ${PXREA_INCLUDE_DIR})
  target_link_libraries(xr_bridge PRIVATE xr_teleop_core ${PXREA_LIBRARY}
    rclcpp::rclcpp rclcpp_components::component ${geometry_msgs_TARGETS} ${std_msgs_TARGETS} ${robot_utils_TARGETS})
  # The SDK lives outside the ROS prefix: keep its directory in the runtime search path.
  set_target_properties(xr_bridge PROPERTIES INSTALL_RPATH "${PXREA_LIBRARY_DIR}" BUILD_RPATH "${PXREA_LIBRARY_DIR}")
  rclcpp_components_register_node(xr_bridge
    PLUGIN "xr_teleop::XrBridgeNode"
    EXECUTABLE xr_bridge_node)
  install(TARGETS xr_bridge
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
else()
  message(WARNING
    "xr_teleop: XRoboToolkit PC Service SDK not found (looked in '${XRT_SDK_DIR}' and "
    "/opt/apps/roboticsservice/SDK). The xr_bridge component is NOT built; install the PC Service "
    "deb or set XRT_SDK_DIR. Core library and tests are still built.")
endif()

ament_python_install_package(xr_teleop)

install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/xr_stub.py DESTINATION lib/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t test_button_event_encoder test_xr_pose_converter test_xr_state_json test_xr_device_selector)
    ament_add_gtest(${t} test/${t}.cpp)
    target_link_libraries(${t} xr_teleop_core nlohmann_json::nlohmann_json)
    target_compile_definitions(${t} PRIVATE XR_TELEOP_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  endforeach()

  if(PXREA_INCLUDE_DIR)
    ament_add_gtest(test_xr_sdk_client test/test_xr_sdk_client.cpp src/xr_sdk_client.cpp)
    target_include_directories(test_xr_sdk_client PRIVATE ${PXREA_INCLUDE_DIR})
    target_link_libraries(test_xr_sdk_client xr_teleop_core)
  endif()

  find_package(ament_cmake_pytest REQUIRED)
  foreach(t test_xr_network)
    ament_add_pytest_test(${t} test/${t}.py
      PYTHON_EXECUTABLE python3
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../../robot_utils/python:${CMAKE_CURRENT_SOURCE_DIR}/../../robot_setup/robot_config)
  endforeach()
endif()

ament_package()
