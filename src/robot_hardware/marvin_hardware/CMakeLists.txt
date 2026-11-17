cmake_minimum_required(VERSION 3.10)
project(marvin_hardware)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_utils REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(robot_comm_core REQUIRED)  # flange ports as buses (tool/arm_bus)
find_package(end_effector_hardware REQUIRED)  # header-only gripper protocol target only (not the plugin library)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN3 REQUIRED eigen3)

# Marvin SDK 是子模块（上游 cynthia-you/TJ_FX_ROBOT_CONTRL_SDK），按臂的底层控制系统固件选一份：
#   343：src/third_party/TJ_FX_ROBOT_CONTRL_SDK_343（固定在 02440e8），用其中的 contrlSDK100343；43 版固件及以上。
#   338：src/third_party/TJ_FX_ROBOT_CONTRL_SDK_338（固定在 1fa6cc6），用其中的 contrlSDK；38 版固件。
# 固件和 SDK 不配套时，连接后读不到控制器版本，驱动随即急停。选择：colcon build --cmake-args -DMARVIN_FIRMWARE=338
set(MARVIN_FIRMWARE "343" CACHE STRING "Marvin control system firmware: 343 or 338")
set_property(CACHE MARVIN_FIRMWARE PROPERTY STRINGS 343 338)
if(MARVIN_FIRMWARE STREQUAL "343")
    set(MARVIN_CONTROL_SDK_DIR "contrlSDK100343")
elseif(MARVIN_FIRMWARE STREQUAL "338")
    set(MARVIN_CONTROL_SDK_DIR "contrlSDK")
else()
    message(FATAL_ERROR "marvin_hardware: MARVIN_FIRMWARE must be 343 or 338 (got '${MARVIN_FIRMWARE}')")
endif()
get_filename_component(_TJ_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/TJ_FX_ROBOT_CONTRL_SDK_${MARVIN_FIRMWARE}" ABSOLUTE)
if(NOT EXISTS "${_TJ_SDK_ROOT}/${MARVIN_CONTROL_SDK_DIR}")
    message(FATAL_ERROR "marvin_hardware: Marvin SDK 子模块未检出（${_TJ_SDK_ROOT}），请执行 git submodule update --init src/third_party/TJ_FX_ROBOT_CONTRL_SDK_${MARVIN_FIRMWARE}")
endif()

set(HEADER_KINE_DIR "${_TJ_SDK_ROOT}/kinematicsSDK/")
set(HEADER_CONTROL_DIR "${_TJ_SDK_ROOT}/${MARVIN_CONTROL_SDK_DIR}/")
set(LIB_KINE_DIR "${_TJ_SDK_ROOT}/kinematicsSDK/")
set(LIB_CONTROL_DIR "${_TJ_SDK_ROOT}/${MARVIN_CONTROL_SDK_DIR}/")

# Build kinematics SDK (libKine.so) if missing, and auto-rebuild if format is invalid.
set(KINE_SO_PATH "${LIB_KINE_DIR}/libKine.so")

# If libKine.so is missing, build it from sources with g++ (Linux .so).
if(NOT EXISTS "${KINE_SO_PATH}")
    add_custom_command(
        OUTPUT "${KINE_SO_PATH}"
        COMMAND /bin/bash -lc "g++ *.cpp -Wall -w -O2 -fPIC -shared -o libKine.so -lpthread -lrt"
        WORKING_DIRECTORY "${LIB_KINE_DIR}"
        COMMENT "Building libKine.so (kinematics SDK, Linux g++)"
        VERBATIM
    )
endif()
add_custom_target(build_kine_sdk ALL DEPENDS "${KINE_SO_PATH}")

# If libKine.so exists but is not a Linux ELF (e.g. a Windows DLL renamed to .so),
# try to rebuild it from source in kinematicsSDK/.
option(MARVIN_AUTO_REBUILD_KINE "Auto rebuild kinematicsSDK libKine.so if format is invalid" ON)
if(MARVIN_AUTO_REBUILD_KINE AND EXISTS "${KINE_SO_PATH}")
    find_program(FILE_CMD file)
    if(FILE_CMD)
        execute_process(
            COMMAND "${FILE_CMD}" -b "${KINE_SO_PATH}"
            OUTPUT_VARIABLE KINE_SO_FILEINFO
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        if(KINE_SO_FILEINFO MATCHES "PE32" OR KINE_SO_FILEINFO MATCHES "MS Windows")
            message(WARNING
                "marvin_hardware: ${KINE_SO_PATH} is not a Linux ELF shared library (${KINE_SO_FILEINFO}). "
                "Attempting to rebuild via direct g++ in ${LIB_KINE_DIR} ..."
            )
            execute_process(
                COMMAND "${CMAKE_COMMAND}" -E rm -f "${KINE_SO_PATH}"
                WORKING_DIRECTORY "${LIB_KINE_DIR}"
                OUTPUT_QUIET
                ERROR_QUIET
            )
            execute_process(
                COMMAND /bin/bash -lc "g++ *.cpp -Wall -w -O2 -fPIC -shared -o libKine.so -lpthread -lrt"
                WORKING_DIRECTORY "${LIB_KINE_DIR}"
                RESULT_VARIABLE KINE_BUILD_RC
                OUTPUT_VARIABLE KINE_BUILD_OUT
                ERROR_VARIABLE KINE_BUILD_ERR
            )
            if(NOT KINE_BUILD_RC EQUAL 0)
                message(FATAL_ERROR
                    "marvin_hardware: failed to rebuild libKine.so via g++ (rc=${KINE_BUILD_RC}).\n"
                    "stdout:\n${KINE_BUILD_OUT}\n"
                    "stderr:\n${KINE_BUILD_ERR}\n"
                    "Please obtain/build a Linux x86_64 ELF libKine.so."
                )
            endif()
            execute_process(
                COMMAND "${FILE_CMD}" -b "${KINE_SO_PATH}"
                OUTPUT_VARIABLE KINE_SO_FILEINFO2
                OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_QUIET
            )
            if(KINE_SO_FILEINFO2 MATCHES "PE32" OR KINE_SO_FILEINFO2 MATCHES "MS Windows")
                message(FATAL_ERROR
                    "marvin_hardware: after rebuild, ${KINE_SO_PATH} is still not a Linux ELF shared library:\n"
                    "  ${KINE_SO_FILEINFO2}\n"
                    "Please obtain/build the correct Linux version of libKine.so."
                )
            else()
                message(STATUS "marvin_hardware: libKine.so rebuilt OK (${KINE_SO_FILEINFO2})")
            endif()
        endif()
    else()
        message(WARNING "marvin_hardware: 'file' command not found; cannot validate/rebuild libKine.so format.")
    endif()
endif()

# build controlSDK
add_custom_command(
    OUTPUT ${LIB_CONTROL_DIR}/libMarvinSDK.so
    COMMAND ${CMAKE_COMMAND} -E env --unset=MAKEFLAGS
            ${CMAKE_MAKE_PROGRAM} -f makefile libMarvinSDK.so
    WORKING_DIRECTORY ${LIB_CONTROL_DIR}
    COMMENT "Building Marvin SDK with Makefile"
)
add_custom_target(build_marvin_sdk ALL DEPENDS ${LIB_CONTROL_DIR}/libMarvinSDK.so)

set(EXTERNAL_KINEMATICSO "${KINE_SO_PATH}")
#
# NOTE:
# Do not hard-fail at *configure time* if libKine.so is missing.
# We deliberately allow building from sources via the build_kine_sdk target,
# so repos can avoid tracking prebuilt .so binaries in git.

## Note: libMarvinSDK.so is built during compilation, so we link it directly via target_link_directories
## instead of using find_library which runs at configure time

## contrlSDK100343 的 CmplOpt.h 仅在 #ifdef CMPL_LIN 下提供 Linux 平台类型定义（SOCKET/LOOPHANDLE 等），
## 因此需要显式定义 CMPL_LIN 宏。
add_compile_definitions(CMPL_LIN)

# Create library
add_library(${PROJECT_NAME} SHARED
        src/marvin_hardware.cpp
        src/sensors/kwr75_marvin485_client.cpp
        src/tool/marvin_rs485_bus.cpp
        src/tool/modbus_io.cpp
        src/tool/arm_bus.cpp
        src/tool/grippers/jodell/jd_gripper.cpp
)

add_dependencies(${PROJECT_NAME} build_kine_sdk build_marvin_sdk)

set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "${LIB_KINE_DIR};${LIB_CONTROL_DIR}"
    INSTALL_RPATH "$ORIGIN"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

target_include_directories(${PROJECT_NAME} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        # "$<BUILD_INTERFACE:${HEADER_KINE_DIR}>"
        # "$<BUILD_INTERFACE:${HEADER_CONTROL_DIR}>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE src
        PRIVATE ${HEADER_KINE_DIR}
        PRIVATE ${HEADER_CONTROL_DIR}
        PRIVATE ${EIGEN3_INCLUDE_DIRS}
)

# Add library directories for linking
target_link_directories(${PROJECT_NAME} PRIVATE
    ${LIB_KINE_DIR}
    ${LIB_CONTROL_DIR}
)

# # Detect architecture and set library path
# if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
#     set(XCORE_LIB_DIR "aarch64")
# else()
#     set(XCORE_LIB_DIR "x86_64")
# endif()

# # Link xCoreSDK libraries
target_link_libraries(${PROJECT_NAME} PUBLIC
        ${hardware_interface_TARGETS}
        ${rclcpp_TARGETS}
        ${robot_utils_TARGETS}
        ${rclcpp_lifecycle_TARGETS}
        ${pluginlib_TARGETS}
        end_effector_hardware::gripper_protocol
        robot_comm_core::robot_comm_bus
        ${EXTERNAL_KINEMATICSO}
        MarvinSDK
        pthread
        rt
)

pluginlib_export_plugin_description_file(hardware_interface marvin_hardware.xml)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    # The flange buses on fake SDK channel functions (no SDK, no arm).
    ament_add_gtest(test_arm_bus test/test_arm_bus.cpp src/tool/arm_bus.cpp)
    target_include_directories(test_arm_bus PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
    target_link_libraries(test_arm_bus robot_comm_core::robot_comm_bus ${rclcpp_TARGETS})
    # End effector components (end_effector_hardware's bus devices) on the flange buses, with simulated devices.
    find_package(robot_comm_modbus REQUIRED)
    ament_add_gtest(test_arm_bus_devices test/test_arm_bus_devices.cpp src/tool/arm_bus.cpp)
    target_include_directories(test_arm_bus_devices PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
    target_link_libraries(test_arm_bus_devices end_effector_hardware::end_effector_hardware
                          robot_comm_core::robot_comm_bus robot_comm_modbus::robot_comm_modbus ${rclcpp_TARGETS})
    ament_add_gtest(test_param_parsing test/test_param_parsing.cpp)
    target_include_directories(test_param_parsing PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    )

endif()

# Minimal example: connect to robot and initialize dual Jodell RG75 gripper (always build if source exists)
set(MARVIN_EXAMPLE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/examples/marvin_gripper_init_example.cpp")
if(EXISTS "${MARVIN_EXAMPLE_SRC}")
    add_executable(marvin_gripper_init_example "${MARVIN_EXAMPLE_SRC}")
    add_dependencies(marvin_gripper_init_example build_kine_sdk build_marvin_sdk)
    target_include_directories(marvin_gripper_init_example PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${HEADER_KINE_DIR}
        ${HEADER_CONTROL_DIR}
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_directories(marvin_gripper_init_example PRIVATE
        ${LIB_KINE_DIR}
        ${LIB_CONTROL_DIR}
    )
    set_target_properties(marvin_gripper_init_example PROPERTIES
        BUILD_RPATH "${LIB_KINE_DIR};${LIB_CONTROL_DIR}"
        INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib"
        INSTALL_RPATH_USE_LINK_PATH TRUE
    )
    target_link_libraries(marvin_gripper_init_example
        ${PROJECT_NAME}
        ${rclcpp_TARGETS}
        end_effector_hardware::gripper_protocol
        ${EXTERNAL_KINEMATICSO}
        MarvinSDK
        pthread
        rt
    )
    install(TARGETS marvin_gripper_init_example
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    message(STATUS "marvin_hardware: building and installing marvin_gripper_init_example")
endif()

# Hand init example: connect to robot and initialize dual LinkerHand (dexterous hand)
set(MARVIN_HAND_EXAMPLE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/examples/marvin_hand_init_example.cpp")
if(EXISTS "${MARVIN_HAND_EXAMPLE_SRC}")
    add_executable(marvin_hand_init_example "${MARVIN_HAND_EXAMPLE_SRC}")
    add_dependencies(marvin_hand_init_example build_kine_sdk build_marvin_sdk)
    target_include_directories(marvin_hand_init_example PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${HEADER_KINE_DIR}
        ${HEADER_CONTROL_DIR}
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_directories(marvin_hand_init_example PRIVATE
        ${LIB_KINE_DIR}
        ${LIB_CONTROL_DIR}
    )
    set_target_properties(marvin_hand_init_example PROPERTIES
        BUILD_RPATH "${LIB_KINE_DIR};${LIB_CONTROL_DIR}"
        INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib"
        INSTALL_RPATH_USE_LINK_PATH TRUE
    )
    target_link_libraries(marvin_hand_init_example
        ${PROJECT_NAME}
        ${rclcpp_TARGETS}
        end_effector_hardware::gripper_protocol
        ${EXTERNAL_KINEMATICSO}
        MarvinSDK
        pthread
        rt
    )
    install(TARGETS marvin_hand_init_example
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    message(STATUS "marvin_hardware: building and installing marvin_hand_init_example")
endif()

# Standalone async raw 485 test: SDK only, no ROS, no marvin_hardware library
set(ASYNC_RAW_SDK_ONLY_SRC "${CMAKE_CURRENT_SOURCE_DIR}/examples/async_raw_485_test_sdk_only.cpp")
if(EXISTS "${ASYNC_RAW_SDK_ONLY_SRC}")
    add_executable(async_raw_485_test_sdk_only "${ASYNC_RAW_SDK_ONLY_SRC}")
    add_dependencies(async_raw_485_test_sdk_only build_marvin_sdk)
    target_include_directories(async_raw_485_test_sdk_only PRIVATE ${HEADER_CONTROL_DIR})
    target_link_directories(async_raw_485_test_sdk_only PRIVATE ${LIB_CONTROL_DIR})
    set_target_properties(async_raw_485_test_sdk_only PROPERTIES
        BUILD_RPATH "${LIB_KINE_DIR};${LIB_CONTROL_DIR}"
        INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib"
        INSTALL_RPATH_USE_LINK_PATH TRUE
    )
    target_link_libraries(async_raw_485_test_sdk_only MarvinSDK pthread)
    install(TARGETS async_raw_485_test_sdk_only
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    message(STATUS "marvin_hardware: building and installing async_raw_485_test_sdk_only (SDK only)")
endif()

# Measurements of the arm's flange RS485 (latency, load, both ports, SDK threading): SDK only, real arm.
set(ARM_RS485_BENCH_SRC "${CMAKE_CURRENT_SOURCE_DIR}/examples/arm_rs485_bench.cpp")
if(EXISTS "${ARM_RS485_BENCH_SRC}")
    add_executable(arm_rs485_bench "${ARM_RS485_BENCH_SRC}")
    add_dependencies(arm_rs485_bench build_marvin_sdk)
    target_include_directories(arm_rs485_bench PRIVATE ${HEADER_CONTROL_DIR})
    target_link_directories(arm_rs485_bench PRIVATE ${LIB_CONTROL_DIR})
    set_target_properties(arm_rs485_bench PROPERTIES
        BUILD_RPATH "${LIB_KINE_DIR};${LIB_CONTROL_DIR}"
        INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib"
        INSTALL_RPATH_USE_LINK_PATH TRUE
    )
    target_link_libraries(arm_rs485_bench MarvinSDK pthread)
    install(TARGETS arm_rs485_bench
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    message(STATUS "marvin_hardware: building and installing arm_rs485_bench (SDK only)")
endif()

install(
        DIRECTORY include/ 
        DESTINATION include/${PROJECT_NAME}
)

install(FILES
    ${LIB_KINE_DIR}/libKine.so
    ${LIB_CONTROL_DIR}/libMarvinSDK.so
    DESTINATION lib
    OPTIONAL
)

# Install Python tools as ROS executables (so `ros2 run marvin_hardware <tool>` works).
install(PROGRAMS
    scripts/tool_dyn_identify_wizard.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME tool_dyn_identify_wizard
)

# Install SDK Python bindings and identification template data.
if(EXISTS "${_TJ_SDK_ROOT}/SDK_PYTHON")
    install(
        DIRECTORY ${_TJ_SDK_ROOT}/SDK_PYTHON
        DESTINATION share/${PROJECT_NAME}/TJ_FX_ROBOT_CONTRL_SDK
        # Upstream vendors MarvinPlatform_* (desktop GUI ELF, ~27MB) next to the Python
        # bindings; it is not required at runtime and must not be installed into the prefix.
        PATTERN "MarvinPlatform*" EXCLUDE
    )
    # Keep SDK_PYTHON unmodified (upstream behavior loads .so next to the .py files),
    # but do NOT track binaries in git: we install the freshly built .so here.
    install(FILES
        ${LIB_KINE_DIR}/libKine.so
        ${LIB_CONTROL_DIR}/libMarvinSDK.so
        DESTINATION share/${PROJECT_NAME}/TJ_FX_ROBOT_CONTRL_SDK/SDK_PYTHON
        OPTIONAL
    )
endif()
# 负载辨识模板数据（上游 CommonConfig/LoadData_ccs；wizard 脚本通过 CommonConfig 读取）。
if(EXISTS "${_TJ_SDK_ROOT}/CommonConfig/LoadData_ccs/LoadData/CfgFile")
    install(
        DIRECTORY ${_TJ_SDK_ROOT}/CommonConfig/LoadData_ccs/LoadData/CfgFile
                  ${_TJ_SDK_ROOT}/CommonConfig/LoadData_ccs/LoadData/IdenTraj
        DESTINATION share/${PROJECT_NAME}/TJ_FX_ROBOT_CONTRL_SDK/CommonConfig/LoadData_ccs/LoadData
    )
endif()

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

ament_export_dependencies(hardware_interface rclcpp rclcpp_lifecycle pluginlib end_effector_hardware)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
