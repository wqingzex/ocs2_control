cmake_minimum_required(VERSION 3.8)
project(robot_comm_modbus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(robot_comm_core REQUIRED)
find_package(robot_comm_serial REQUIRED)

find_library(MODBUS_LIBRARY NAMES modbus PATHS /usr/lib /usr/local/lib)
if(NOT MODBUS_LIBRARY)
  message(FATAL_ERROR "libmodbus not found. Please install libmodbus-dev")
endif()

add_library(${PROJECT_NAME} SHARED
        src/libmodbus_rtu_master.cpp
        src/mock_modbus_master.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC robot_comm_core::robot_comm_core robot_comm_serial::robot_comm_serial)
# libmodbus stays out of the public interface: callers only see ModbusMaster.
target_link_libraries(${PROJECT_NAME} PRIVATE ${MODBUS_LIBRARY})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mock_modbus_master test/test_mock_modbus_master.cpp)
  target_link_libraries(test_mock_modbus_master ${PROJECT_NAME})
  ament_add_gtest(test_libmodbus_rtu_master test/test_libmodbus_rtu_master.cpp)
  target_link_libraries(test_libmodbus_rtu_master ${PROJECT_NAME})
  ament_add_gtest(test_rtu_frame test/test_rtu_frame.cpp)
  target_link_libraries(test_rtu_frame ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(robot_comm_core robot_comm_serial)
ament_package()
