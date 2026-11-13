cmake_minimum_required(VERSION 3.8)
project(robot_comm_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(robot_comm_core REQUIRED)

add_library(${PROJECT_NAME} SHARED
        src/socket_can_bus.cpp
        src/mock_can_bus.cpp
        src/interface_configurators.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC robot_comm_core::robot_comm_core)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mock_can_bus test/test_mock_can_bus.cpp)
  target_link_libraries(test_mock_can_bus ${PROJECT_NAME})
  ament_add_gtest(test_interface_configurators test/test_interface_configurators.cpp)
  target_link_libraries(test_interface_configurators ${PROJECT_NAME})

  # The command the driver runs and the sudoers rule that scripts/setup.sh writes must stay the same text.
  set(_SETUP_SH "${CMAKE_CURRENT_SOURCE_DIR}/../../../../scripts/setup.sh")
  if(EXISTS "${_SETUP_SH}")
    add_test(NAME sudoers_rule_matches_the_driver_command
            COMMAND grep -qF "up type can bitrate 1000000 dbitrate 5000000 restart-ms 100 fd on" "${_SETUP_SH}")
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(robot_comm_core)
ament_package()
