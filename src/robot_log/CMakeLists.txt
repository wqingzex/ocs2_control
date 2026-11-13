cmake_minimum_required(VERSION 3.14)
project(robot_log)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)

# Header only: everything is inline in robot_log/log_mode.hpp.
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME} INTERFACE
        ${rclcpp_TARGETS}
        rcutils::rcutils
        ${std_msgs_TARGETS})

# Offline tools (fault code registry, flight recorder decoder): python/robot_log_tools, no ROS needed to use them.
ament_python_install_package(robot_log_tools PACKAGE_DIR python/robot_log_tools)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_log_mode test/test_log_mode.cpp)
    target_link_libraries(test_log_mode ${PROJECT_NAME})
    ament_add_gtest(test_health test/test_health.cpp)
    target_link_libraries(test_health ${PROJECT_NAME})
    ament_add_gtest(test_flight test/test_flight.cpp)
    target_link_libraries(test_flight ${PROJECT_NAME} pthread)

    find_package(ament_cmake_pytest REQUIRED)
    foreach(t test_fault_codes test_flight_decoder test_analyze test_log test_evidence)
        ament_add_pytest_test(${t} test/${t}.py
            PYTHON_EXECUTABLE python3
            APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python)
    endforeach()
endif ()

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp rcutils std_msgs)
ament_package()
