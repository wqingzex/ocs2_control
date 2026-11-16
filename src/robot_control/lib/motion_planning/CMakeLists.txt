cmake_minimum_required(VERSION 3.14)
project(motion_planning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)

# 创建单个库
# 使用 STATIC 库，但添加 POSITION_INDEPENDENT_CODE 以便可以链接到共享库
add_library(${PROJECT_NAME} STATIC
        src/common/joint_vector.cpp
        src/common/normalization_point.cpp
        src/common/trajectory_init_parameters.cpp
        src/common/quaternion.cpp
        src/common/rotation_matrix.cpp
        src/common/frame.cpp
        src/path_planner/bezier_curve.cpp
        src/path_planner/movej.cpp src/path_planner/movel.cpp
        src/path_planner/smooth_curve_of_multi_joints_using_blending.cpp
        src/path_planner/circular_curve.cpp
        src/path_planner/speedj.cpp
        src/velocity_planner/doubles_with_zero_boundary_of_vel_and_acc.cpp
        src/velocity_planner/doubles.cpp
        src/velocity_planner/plan_velocity_using_doubles.cpp
        src/kinematics/fa_w2_fk.cpp
        src/kinematics/fa_w2_ik.cpp
)

# 设置位置无关代码，以便静态库可以链接到共享库
set_target_properties(${PROJECT_NAME} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
)

# 头文件放在 include/motion_planning/ 下，构建时和安装后都写 #include "motion_planning/<模块>/<头文件>"
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${rclcpp_TARGETS})

# 只安装公共 API 头文件，隐藏内部实现细节
# 使用前向声明后，不再需要安装内部实现细节的头文件

# 公共 API: path_planner 目录下的所有头文件
install(
        DIRECTORY include/${PROJECT_NAME}/path_planner/
        DESTINATION include/${PROJECT_NAME}/path_planner
        FILES_MATCHING PATTERN "*.h"
)
# 公共 API: kinematics 目录下的所有头文件
install(
        DIRECTORY include/${PROJECT_NAME}/kinematics/
        DESTINATION include/${PROJECT_NAME}/kinematics
        FILES_MATCHING PATTERN "*.h"
)
# 公共类型定义: common 目录下的公共类型头文件
install(
        FILES include/${PROJECT_NAME}/common/joint_vector.h
        include/${PROJECT_NAME}/common/trajectory_init_parameters.h
        include/${PROJECT_NAME}/common/quaternion.h
        include/${PROJECT_NAME}/common/rotation_matrix.h
        include/${PROJECT_NAME}/common/frame.h
        include/${PROJECT_NAME}/common/logger.h
        DESTINATION include/${PROJECT_NAME}/common
)

# 注意: velocity_planner 和 normalization_point 现在是内部实现细节
# 通过前向声明，它们不再需要被安装
install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_logging test/test_logging.cpp)
    target_link_libraries(test_logging ${PROJECT_NAME})
endif()

# 导出为 ament 包（关键步骤）
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 rclcpp)
ament_export_include_directories("include")

# 生成 ament 包配置
ament_package()