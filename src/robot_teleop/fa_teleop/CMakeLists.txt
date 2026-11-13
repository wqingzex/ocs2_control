cmake_minimum_required(VERSION 3.16)
project(fa_teleop LANGUAGES CXX)

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
find_package(robot_log REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

# Double-click entry in the workspace root, built natively on both x86_64 and ARM64.
get_filename_component(FA_TELEOP_WS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
add_executable(w2_console_launcher src/console_launcher.cpp)
set_target_properties(w2_console_launcher PROPERTIES
  OUTPUT_NAME "W2控制台"
  RUNTIME_OUTPUT_DIRECTORY "${FA_TELEOP_WS_ROOT}")
# Use an ELF executable rather than a PIE shared-object MIME type in desktop file managers.
target_link_options(w2_console_launcher PRIVATE -no-pie)

# ---------------------------------------------------------------------------
# Core logic (no SDK, no ROS): JSON parsing, pose conversion, button events.
# ---------------------------------------------------------------------------
add_library(fa_teleop_core STATIC
  src/xr_state_json.cpp
  src/xr_pose_converter.cpp
  src/button_event_encoder.cpp
  src/simple_button_encoder.cpp)
target_include_directories(fa_teleop_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(fa_teleop_core PUBLIC Eigen3::Eigen PRIVATE nlohmann_json::nlohmann_json)
set_target_properties(fa_teleop_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# XRoboToolkit PC Service SDK (installed by the PC Service deb); override with -DXRT_SDK_DIR=<dir>
# or env XRT_SDK_DIR (dir containing include/ and x64|aarch64/).
set(XRT_SDK_DIR "$ENV{XRT_SDK_DIR}" CACHE PATH "XRoboToolkit PXREARobotSDK root")
set(_xrt_hints ${XRT_SDK_DIR} /opt/apps/roboticsservice/SDK)
find_path(PXREA_INCLUDE_DIR PXREARobotSDK.h HINTS ${_xrt_hints} PATH_SUFFIXES include NO_DEFAULT_PATH)
find_library(PXREA_LIBRARY PXREARobotSDK HINTS ${_xrt_hints}
  PATH_SUFFIXES x64 aarch64 arm64 linux/64 linux_aarch64/64 lib NO_DEFAULT_PATH)

if(PXREA_INCLUDE_DIR AND PXREA_LIBRARY)
  message(STATUS "fa_teleop: XRoboToolkit SDK found: ${PXREA_LIBRARY}")
  get_filename_component(PXREA_LIBRARY_DIR "${PXREA_LIBRARY}" DIRECTORY)

  add_library(xr_bridge SHARED src/xr_sdk_client.cpp src/xr_bridge_node.cpp)
  target_include_directories(xr_bridge PRIVATE ${PXREA_INCLUDE_DIR})
  target_link_libraries(xr_bridge PRIVATE fa_teleop_core ${PXREA_LIBRARY}
    rclcpp::rclcpp rclcpp_components::component ${geometry_msgs_TARGETS} ${std_msgs_TARGETS} ${robot_log_TARGETS})
  # The SDK lives outside the ROS prefix: keep its directory in the runtime search path.
  set_target_properties(xr_bridge PROPERTIES INSTALL_RPATH "${PXREA_LIBRARY_DIR}" BUILD_RPATH "${PXREA_LIBRARY_DIR}")
  rclcpp_components_register_node(xr_bridge
    PLUGIN "fa_teleop::XrBridgeNode"
    EXECUTABLE xr_bridge_node)
  install(TARGETS xr_bridge
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
else()
  message(WARNING
    "fa_teleop: XRoboToolkit PC Service SDK not found (looked in '${XRT_SDK_DIR}' and "
    "/opt/apps/roboticsservice/SDK). The xr_bridge component is NOT built; install the PC Service "
    "deb or set XRT_SDK_DIR. Core library and tests are still built.")
endif()

# Launch helpers (config validation, camera specs, controller args) used by launch/*.py and start.sh
ament_python_install_package(fa_teleop_runtime)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(PROGRAMS scripts/xr_stub.py DESTINATION lib/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t test_button_event_encoder test_simple_button_encoder test_xr_pose_converter test_xr_state_json test_xr_device_selector)
    ament_add_gtest(${t} test/${t}.cpp)
    target_link_libraries(${t} fa_teleop_core nlohmann_json::nlohmann_json)
    target_compile_definitions(${t} PRIVATE FA_TELEOP_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  endforeach()

  if(PXREA_INCLUDE_DIR)
    ament_add_gtest(test_xr_sdk_client test/test_xr_sdk_client.cpp src/xr_sdk_client.cpp)
    target_include_directories(test_xr_sdk_client PRIVATE ${PXREA_INCLUDE_DIR})
    target_link_libraries(test_xr_sdk_client fa_teleop_core)
  endif()

  find_package(ament_cmake_pytest REQUIRED)
  foreach(t test_launch_config test_units_supervisor test_init_steps test_console test_navigation test_xr_network)
    ament_add_pytest_test(${t} test/${t}.py
      PYTHON_EXECUTABLE python3
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  endforeach()
  ament_add_pytest_test(test_console_launcher test/test_console_launcher.py
    PYTHON_EXECUTABLE python3
    ENV W2_CONSOLE_LAUNCHER=$<TARGET_FILE:w2_console_launcher>)
endif()

ament_package()
