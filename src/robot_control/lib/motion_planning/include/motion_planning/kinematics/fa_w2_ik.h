#pragma once
#include "motion_planning/common/frame.h"
#include <array>

namespace planning
{
    class FaW2IK
    {
    public:
        FaW2IK();
        ~FaW2IK() = default;
        //增加腰部电机逆解，有需要再增加其他部分
        bool ik_of_body_joint4(const Eigen::Vector4d& q_in, const Frame& f,
                               Eigen::Vector4d& q_out);

    private:
        // 从基坐标系到body_base的坐标系
        Frame body_base;
        // 腰部四个电机的修正DH参数
        Eigen::Vector4d a_body;
        Eigen::Vector4d d_body;
        Eigen::Vector4d alpha_body;
        Eigen::Vector4d theta_offset;
        Eigen::Vector4d theta_direction;
        // 求解从body_base到body_joint4的两个解析解
        bool full_solvers_of_body_base_to_body_joint4(
            const Frame& f, std::array<Eigen::Vector4d, 2>& solutions);
        Eigen::Vector4d choose_nearest_solution_of_body_joint4(
            const Eigen::Vector4d& q0, std::array<Eigen::Vector4d, 2>& solutions);
        double min_val = 1.0e-6;
    };
} // namespace planning
