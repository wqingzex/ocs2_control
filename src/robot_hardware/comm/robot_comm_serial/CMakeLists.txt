cmake_minimum_required(VERSION 3.8)
project(robot_comm_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(robot_comm_core REQUIRED)

add_library(${PROJECT_NAME} SHARED
        src/posix_serial_port.cpp
        src/usb_latency.cpp
        src/mock_serial_port.cpp)
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
  ament_add_gtest(test_posix_serial_port test/test_posix_serial_port.cpp)
  target_link_libraries(test_posix_serial_port ${PROJECT_NAME})
  ament_add_gtest(test_mock_serial_port test/test_mock_serial_port.cpp)
  target_link_libraries(test_mock_serial_port ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(robot_comm_core)
ament_package()
