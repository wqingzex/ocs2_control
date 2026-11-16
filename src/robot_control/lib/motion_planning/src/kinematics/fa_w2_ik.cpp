#include "motion_planning/kinematics/fa_w2_ik.h"

namespace planning {
FaW2IK::FaW2IK() {
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
};

bool FaW2IK::ik_of_body_joint4(const Eigen::Vector4d &q_in,
                                     const Frame &f, Eigen::Vector4d &q_out) {
  Frame body_base_to_body_joint4;
  body_base_to_body_joint4 = body_base.Inverse() * f;

  std::array<Eigen::Vector4d, 2> solutions;
  if (!full_solvers_of_body_base_to_body_joint4(body_base_to_body_joint4,
                                                solutions)) {
    return false;
  }

  q_out = choose_nearest_solution_of_body_joint4(q_in, solutions);
  return true;
};

bool FaW2IK::full_solvers_of_body_base_to_body_joint4(
    const Frame &f, std::array<Eigen::Vector4d, 2> &solutions) {
  Eigen::Vector4d joint_angle1, joint_angle2;
  // 求第四个关节
  double th_4 = atan2(f.rot(1, 0), f.rot(1, 1));
  // 第四个关节的坐标系
  Frame f4 = Frame::mdh(th_4, d_body(3), a_body(3), alpha_body(3));
  Frame f1_3 = f * f4.Inverse();
  // 前三个关节的和
  double cos_123 = f1_3.rot(0, 0);
  double sin_123 = -f1_3.rot(0, 1);
  double th_123 = atan2(sin_123, cos_123);
  // 求出两个关节2的值
  double cos_2 = (f1_3.p(0) * f1_3.p(0) + f1_3.p(2) * f1_3.p(2) -
                  a_body(1) * a_body(1) - a_body(2) * a_body(2)) /
                 (2.0 * a_body(1) * a_body(2));
  double sin_2 = sqrt(1.0 - cos_2 * cos_2);

  double th_2_1 = atan2(sin_2, cos_2);
  double th_2_2 = atan2(-sin_2, cos_2);

  // 求关节1和关节3的值
  double th1_add_gama = atan2(-f1_3.p(2), f1_3.p(0));
  if (fabs(f1_3.p(2)) < min_val && fabs(f1_3.p(0)) < min_val) {
    return false;
  }

  // 当th2=th_2_1时
  double k_11 = a_body(2) * cos(th_2_1) + a_body(1);
  double k_21 = a_body(2) * sin(th_2_1);
  double th_1_1 = th1_add_gama - atan2(k_21, k_11);
  double th_3_1 = th_123 - th_1_1 - th_2_1;
  // 当th2=th_2_2时
  double k_12 = a_body(2) * cos(th_2_2) + a_body(1);
  double k_22 = a_body(2) * sin(th_2_2);
  double th_1_2 = th1_add_gama - atan2(k_22, k_12);
  double th_3_2 = th_123 - th_1_2 - th_2_2;
  joint_angle1 << th_1_1, th_2_1, th_3_1, th_4;
  joint_angle2 << th_1_2, th_2_2, th_3_2, th_4;
  for (size_t i = 0; i < 4; i++) {
    joint_angle1(i) -= theta_offset(i);
    joint_angle1(i) *= theta_direction(i);
    joint_angle2(i) -= theta_offset(i);
    joint_angle2(i) *= theta_direction(i);
  }

  solutions[0] = joint_angle1;
  solutions[1] = joint_angle2;
  return true;
};

Eigen::Vector4d FaW2IK::choose_nearest_solution_of_body_joint4(
    const Eigen::Vector4d &q0, std::array<Eigen::Vector4d, 2> &solutions) {
  Eigen::Vector4d res;
  double d1 = 0;
  double d2 = 0;
  for (size_t i = 0; i < 4; i++) {
    d1 += (q0(i) - solutions[0](i)) * (q0(i) - solutions[0](i));
    d2 += (q0(i) - solutions[1](i)) * (q0(i) - solutions[1](i));
  }
  if (d1 < d2) {
    res = solutions[0];
  } else {
    res = solutions[1];
  }
  return res;
};

} // namespace planning
