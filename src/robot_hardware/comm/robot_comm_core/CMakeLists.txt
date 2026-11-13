cmake_minimum_required(VERSION 3.8)
project(robot_comm_core)

find_package(ament_cmake REQUIRED)

# Header-only: interfaces and small pure helpers.
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_interface_name test/test_interface_name.cpp)
  target_link_libraries(test_interface_name ${PROJECT_NAME})
  ament_add_gtest(test_can_config test/test_can_config.cpp)
  target_link_libraries(test_can_config ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
