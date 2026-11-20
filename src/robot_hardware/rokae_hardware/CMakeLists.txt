cmake_minimum_required(VERSION 3.8)
project(rokae_hardware)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
pkg_check_modules(OROCOS_KDL REQUIRED orocos-kdl)

# 珞石 xCoreSDK：头文件来自子模块 third_party/xCoreSDK-CPP，预编译库由 ./scripts/setup.sh deps 从官方 Release 下载
set(ROKAE_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/xCoreSDK-CPP" CACHE PATH "xCoreSDK-CPP checkout")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ROKAE_LIB_DIR "aarch64")
else()
    set(ROKAE_LIB_DIR "x86_64")
endif()
set(ROKAE_LIB_PATH "${ROKAE_SDK_DIR}/lib/Linux/${ROKAE_LIB_DIR}")
if(NOT EXISTS "${ROKAE_SDK_DIR}/include/rokae/robot.h")
    message(FATAL_ERROR "xCoreSDK-CPP not found in ${ROKAE_SDK_DIR}: run ./scripts/setup.sh deps (git submodule update --init)")
endif()
foreach(_rokae_lib libxCoreSDK.a libxMateModel.a)
    if(NOT EXISTS "${ROKAE_LIB_PATH}/${_rokae_lib}")
        message(FATAL_ERROR "${ROKAE_LIB_PATH}/${_rokae_lib} not found: run ./scripts/setup.sh deps "
                "(downloads the xCoreSDK release package for ${ROKAE_LIB_DIR})")
    endif()
endforeach()

# Create library
add_library(${PROJECT_NAME} SHARED
        src/rokae_hardware.cpp
        src/jodell_gripper.cpp
        src/linkerhand_l6.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE src
        PRIVATE ${ROKAE_SDK_DIR}/include
        PRIVATE ${EIGEN3_INCLUDE_DIRS}
        PRIVATE ${OROCOS_KDL_INCLUDE_DIRS}
)

# Link xCoreSDK (static)
target_link_libraries(${PROJECT_NAME} PUBLIC
        ${hardware_interface_TARGETS}
        ${rclcpp_TARGETS}
        ${rclcpp_lifecycle_TARGETS}
        ${pluginlib_TARGETS}
        ${ROKAE_LIB_PATH}/libxCoreSDK.a
        ${ROKAE_LIB_PATH}/libxMateModel.a
        ${EIGEN3_LIBRARIES}
        ${OROCOS_KDL_LIBRARIES}
        pthread
        rt
)

pluginlib_export_plugin_description_file(hardware_interface rokae_hardware.xml)

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

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_export_dependencies(hardware_interface rclcpp rclcpp_lifecycle pluginlib)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
