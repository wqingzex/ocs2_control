cmake_minimum_required(VERSION 3.8)
project(robot_comm_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

# Header-only: interfaces and small pure helpers.
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

# Buses (bus.h): the queueing every bus shares and the process-wide BusHub. A shared library, not header-only, so
# that every plugin loaded into one ros2_control_node sees the same BusHub.
add_library(robot_comm_bus SHARED
        src/queued_bus.cpp
        src/bus_hub.cpp)
target_link_libraries(robot_comm_bus PUBLIC ${PROJECT_NAME} Threads::Threads)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} robot_comm_bus EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_interface_name test/test_interface_name.cpp)
  target_link_libraries(test_interface_name ${PROJECT_NAME})
  ament_add_gtest(test_can_config test/test_can_config.cpp)
  target_link_libraries(test_can_config ${PROJECT_NAME})
  ament_add_gtest(test_bus_name test/test_bus_name.cpp)
  target_link_libraries(test_bus_name ${PROJECT_NAME})
  ament_add_gtest(test_queued_bus test/test_queued_bus.cpp)
  target_link_libraries(test_queued_bus robot_comm_bus)
  ament_add_gtest(test_bus_hub test/test_bus_hub.cpp)
  target_link_libraries(test_bus_hub robot_comm_bus)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Threads)
ament_package()
