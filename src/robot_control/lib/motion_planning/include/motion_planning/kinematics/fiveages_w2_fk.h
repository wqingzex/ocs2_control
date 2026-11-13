#pragma once
#include "motion_planning/common/frame.h"

namespace planning
{
    class FiveAgesW2FK
    {
    public:
        FiveAgesW2FK();
        // 增加腰部电机正解，后续有需要再增加其他部分
        Frame base_to_body_joint4(Eigen::Vector4d theta);
        ~FiveAgesW2FK() = default;

    private:
        Frame body_base_to_body_joint4(Eigen::Vector4d theta);
        // 从基坐标系到body_base的坐标系
        Frame body_base;
        // 腰部四个电机的修正DH参数
        Eigen::Vector4d a_body;
        Eigen::Vector4d d_body;
        Eigen::Vector4d alpha_body;
        Eigen::Vector4d theta_offset;
        Eigen::Vector4d theta_direction;
    };
} // namespace planning
