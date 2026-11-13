#include "motion_planning/kinematics/fiveages_w2_fk.h"
#include "motion_planning/common/frame.h"

namespace planning
{
    FiveAgesW2FK::FiveAgesW2FK()
    {
        // 从基坐标系到body_base的坐标系
        RotationMatrix rot_identy(Eigen::Matrix3d::Identity());
        Eigen::Vector3d base_footprint_joint_p(0.0, 0.0, 0.04);
        Eigen::Vector3d body_fixed_joint_p(0.10525, 0.0, 0.2295);
        Frame base_footprint_joint(rot_identy, base_footprint_joint_p);
        Frame body_fixed_joint(rot_identy, body_fixed_joint_p);
        body_base = base_footprint_joint * body_fixed_joint;
        // 腰部的修正dh参数：
        a_body(0) = 0.0;
        a_body(1) = 0.322;
        a_body(2) = 0.355;
        a_body(3) = -7.1612e-5;

        d_body(0) = 0.0;
        d_body(1) = 0.0;
        d_body(2) = 0.0;
        d_body(3) = 0.14331;

        alpha_body(0) = -M_PI_2;
        alpha_body(1) = 0.0;
        alpha_body(2) = 0.0;
        alpha_body(3) = M_PI_2;

        theta_offset(0) = -M_PI_2;
        theta_offset(1) = 0.0;
        theta_offset(2) = M_PI_2;
        theta_offset(3) = 0.0;

        theta_direction(0) = 1.0;
        theta_direction(1) = -1.0;
        theta_direction(2) = 1.0;
        theta_direction(3) = -1.0;
    }

    Frame FiveAgesW2FK::body_base_to_body_joint4(Eigen::Vector4d theta)
    {
        Frame res, tmp;
        tmp = Frame::Zero();
        for (size_t i = 0; i < 4; i++)
        {
            double tmp_th = theta(i) * theta_direction(i) + theta_offset(i);
            Frame current_frame =
                Frame::mdh(tmp_th, d_body[i], a_body[i], alpha_body[i]);
            res = tmp * current_frame;
            tmp = res;
        }
        res.q = Quaternion::convertRotationMatrixToQuaternion(res.rot.data);
        return res;
    };

    Frame FiveAgesW2FK::base_to_body_joint4(Eigen::Vector4d theta)
    {
        Frame res, tmp;
        tmp = body_base_to_body_joint4(theta);
        res = body_base * tmp;
        res.q = Quaternion::convertRotationMatrixToQuaternion(res.rot.data);
        return res;
    };
} // namespace planning
