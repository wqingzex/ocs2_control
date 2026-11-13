#include "motion_planning/common/quaternion.h"

namespace planning {
Quaternion::Quaternion(double _w, double _x, double _y, double _z)
    : w(_w), x(_x), y(_y), z(_z) {}

Quaternion::Quaternion(const Quaternion &arg)
    : w(arg.w), x(arg.x), y(arg.y), z(arg.z) {}

void Quaternion::checkTargetPointWithShortCurve(Quaternion &target) {
  double cos_omega = dot(target);
  if (cos_omega < 0.0) {
    target = target * (-1.0);
  }
}

Quaternion Quaternion::slerp(const Quaternion &target, double h) {
  Quaternion res;
  // checkTargetPointWithShortCurve(target);
  double omega = acos(dot(target));
  if (fabs(omega) < min_val) {
    res = *this;
  } else {
    res = *this * (inverse() * target).pow(h);
  }
  return res;
};

Quaternion Quaternion::rotateAroundAxisByTheta(const Eigen::Vector3d axis,
                                               const double theta,
                                               bool is_body_system) {
  Quaternion tmp_q(0.0, axis(0), axis(1), axis(2));
  tmp_q =
      tmp_q * theta * 0.5; // 因为四元数旋转的角度是2吧倍的轴角角度，所以要乘0.5
  Quaternion exp_q = tmp_q.exp();
  Quaternion res;
  if (is_body_system) {
    res = *this * exp_q;
  } else {
    res = exp_q * (*this);
  }
  return res;
};

Quaternion &Quaternion::operator=(const Quaternion &arg) {
  w = arg.w;
  x = arg.x;
  y = arg.y;
  z = arg.z;
  return *this;
}

Quaternion Quaternion::operator+(const Quaternion &arg) const {
  return Quaternion(w + arg.w, x + arg.x, y + arg.y, z + arg.z);
}
Quaternion Quaternion::operator-(const Quaternion &arg) const {
  return Quaternion(w - arg.w, x - arg.x, y - arg.y, z - arg.z);
};
Quaternion Quaternion::operator*(const Quaternion &arg) const {
  Quaternion res;
  res.w =
      this->w * arg.w - (this->x * arg.x + this->y * arg.y + this->z * arg.z);
  res.x = this->w * arg.x + arg.w * this->x + this->y * arg.z - this->z * arg.y;
  res.y = this->w * arg.y + arg.w * this->y + this->z * arg.x - this->x * arg.z;
  res.z = this->w * arg.z + arg.w * this->z + this->x * arg.y - this->y * arg.x;
  return res;
};
Quaternion Quaternion::operator*(const double arg) const {
  return Quaternion(w * arg, x * arg, y * arg, z * arg);
};

double Quaternion::norm() const { return sqrt(w * w + x * x + y * y + z * z); };

Quaternion Quaternion::normalize() const {
  double num = norm();
  Quaternion res(*this);
  if (num > min_val) {
    return res * (1.0 / num);
  } else {
    return Quaternion(0.0, 0.0, 0.0, 0.0);
  }
};

void Quaternion::normalized() {
  double num = norm();
  if (num > min_val) {
    w /= num;
    x /= num;
    y /= num;
    z /= num;
  } else {
    w = 0.0;
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }
};

double Quaternion::dot(const Quaternion &arg) const {
  return this->w * arg.w + this->x * arg.x + this->y * arg.y + this->z * arg.z;
}

Quaternion Quaternion::conj() const { return Quaternion(w, -x, -y, -z); }
Quaternion Quaternion::inverse() const {
  Quaternion res;
  res = conj() * (1.0 / std::pow(norm(), 2.0));
  return res;
};
Quaternion Quaternion::exp() const {
  Quaternion res;
  // 正常情况下四元数求exp是对【0,，x,y,z】求的
  if (fabs(x) < min_val && fabs(y) < min_val && fabs(z) < min_val) {
    res.w = 1.0;
    res.x = 0.0;
    res.y = 0.0;
    res.z = 0.0;
  } else {
    double theta = sqrt(x * x + y * y + z * z);
    res.w = cos(theta);
    res.x = x * sin(theta) / theta;
    res.y = y * sin(theta) / theta;
    res.z = z * sin(theta) / theta;
  }
  // 如果是[w,x,y,z]则要加上对数值w的exp
  res = res * std::exp(w);
  return res;
};
Quaternion Quaternion::log() const {
  Quaternion res;
  double q_norm = norm();
  double log_q_norm = std::log(q_norm);
  if ((fabs(x) < min_val && fabs(y) < min_val && fabs(z) < min_val) ||
      q_norm < min_val) {
    res.x = 0.0;
    res.y = 0.0;
    res.z = 0.0;
  } else {
    double theta = acos(w / q_norm);
    double sum_v = sqrt(x * x + y * y + z * z);
    res.x = theta * x / sum_v;
    res.y = theta * y / sum_v;
    res.z = theta * z / sum_v;
  }
  // 如果是单位四元数求log后w=0,不是单位四元数就有一个值
  res.w = log_q_norm;
  return res;
};
Quaternion Quaternion::pow(const double t) const {
  double norm_val = norm();
  Quaternion res;
  res = (normalize().log() * t).exp();
  res = res * std::pow(norm_val, t);
  return res;
};

Eigen::Matrix3d Quaternion::toRotationMatrix() const {
  Eigen::Matrix3d res;
  double q_norm = norm();
  if (q_norm < min_val) {
    res.setIdentity();
  } else {
    res(0, 0) = 2.0 * (w * w + x * x) - 1.0;
    res(0, 1) = 2.0 * (x * y - w * z);
    res(0, 2) = 2.0 * (x * z + w * y);
    res(1, 0) = 2.0 * (x * y + w * z);
    res(1, 1) = 2.0 * (w * w + y * y) - 1.0;
    res(1, 2) = 2.0 * (y * z - w * x);
    res(2, 0) = 2.0 * (x * z - w * y);
    res(2, 1) = 2.0 * (y * z + w * x);
    res(2, 2) = 2.0 * (w * w + z * z) - 1.0;
  }
  return res;
};

Quaternion
Quaternion::convertRotationMatrixToQuaternion(const Eigen::Matrix3d &rot) {
  Quaternion res;
  double kMathEpsilon = 1.0e-9;
  const double trace = rot(0, 0) + rot(1, 1) + rot(2, 2);
  Eigen::Vector3d img;
  if (trace > kMathEpsilon) {
    double s = std::sqrt(trace + 1.0);
    res.w = 0.5 * s;
    s = 0.5 / s;
    img(0) = (rot(2, 1) - rot(1, 2)) * s;
    img(1) = (rot(0, 2) - rot(2, 0)) * s;
    img(2) = (rot(1, 0) - rot(0, 1)) * s;

  } else {
    const int i = rot(0, 0) < rot(1, 1) ? (rot(1, 1) < rot(2, 2) ? 2 : 1)
                                        : (rot(0, 0) < rot(2, 2) ? 2 : 0);
    const int j = (i + 1) % 3;
    const int k = (i + 2) % 3;

    double s = std::sqrt(rot(i, i) - rot(j, j) - rot(k, k) + 1);
    img(i) = 0.5 * s;
    s = 0.5 / s;
    res.w = (rot(k, j) - rot(j, k)) * s;
    img(j) = (rot(j, i) + rot(i, j)) * s;
    img(k) = (rot(k, i) + rot(i, k)) * s;
  }
  res.x = img(0);
  res.y = img(1);
  res.z = img(2);
  return res;
};

QuaternionPoint::QuaternionPoint(const QuaternionPoint &arg)
    : q(arg.q), qd(arg.qd), qdd(arg.qdd), qddd(arg.qddd), rot(arg.rot),
      angle_vel_in_body_system(arg.angle_vel_in_body_system),
      angle_acc_in_body_system(arg.angle_acc_in_body_system),
      angle_jerk_in_body_system(arg.angle_jerk_in_body_system),
      angle_vel_in_space_system(arg.angle_vel_in_space_system),
      angle_acc_in_space_system(arg.angle_acc_in_space_system),
      angle_jerk_in_space_system(arg.angle_jerk_in_space_system) {}

QuaternionPoint &QuaternionPoint::operator=(const QuaternionPoint &arg) {
  q = arg.q;
  qd = arg.qd;
  qdd = arg.qdd;
  qddd = arg.qddd;
  rot = arg.rot;
  angle_vel_in_body_system = arg.angle_vel_in_body_system;
  angle_acc_in_body_system = arg.angle_acc_in_body_system;
  angle_jerk_in_body_system = arg.angle_jerk_in_body_system;
  angle_vel_in_space_system = arg.angle_vel_in_space_system;
  angle_acc_in_space_system = arg.angle_acc_in_space_system;
  angle_jerk_in_space_system = arg.angle_jerk_in_space_system;
  return *this;
};

QuaternionPoint QuaternionPoint::slerp(const Quaternion &target,
                                       const double h) {
  QuaternionPoint res;
  res.q = q.slerp(target, h);
  return res;
};
QuaternionPoint QuaternionPoint::slerp(const Quaternion &target, const double h,
                                       const double hdot) {
  QuaternionPoint res;
  res.q = q.slerp(target, h);
  Quaternion log_inv_q0_q1;
  log_inv_q0_q1 = (q.inverse() * target).log();
  res.qd = res.q * log_inv_q0_q1 * hdot;
  return res;
};
QuaternionPoint QuaternionPoint::slerp(const Quaternion &target, const double h,
                                       const double hdot,
                                       const double hdotdot) {
  QuaternionPoint res;
  res.q = q.slerp(target, h);
  Quaternion log_inv_q0_q1, log_inv_q0_q1_pow2, dq_dh, d2q_dh2;
  log_inv_q0_q1 = (q.inverse() * target).log();
  log_inv_q0_q1_pow2 = log_inv_q0_q1 * log_inv_q0_q1;
  dq_dh=res.q*log_inv_q0_q1;
  d2q_dh2=res.q*log_inv_q0_q1_pow2;

  res.qd = dq_dh* hdot;
  res.qdd = d2q_dh2* (hdot*hdot) + dq_dh * hdotdot;
  return res;
};
QuaternionPoint QuaternionPoint::slerp(const Quaternion &target, const double h,
                                       const double hdot, const double hdotdot,
                                       const double hdotdotdot) {
  QuaternionPoint res;
  res.q = q.slerp(target, h);
  Quaternion log_inv_q0_q1, log_inv_q0_q1_pow2, log_inv_q0_q1_pow3,dq_dh,d2q_dh2,d3q_dh3;
  log_inv_q0_q1 = (q.inverse() * target).log();
  log_inv_q0_q1_pow2 = log_inv_q0_q1 * log_inv_q0_q1;
  log_inv_q0_q1_pow3 = log_inv_q0_q1_pow2 * log_inv_q0_q1;
  dq_dh=res.q*log_inv_q0_q1;
  d2q_dh2=res.q*log_inv_q0_q1_pow2;
  d3q_dh3=res.q*log_inv_q0_q1_pow3;

  res.qd = dq_dh* hdot;
  res.qdd = d2q_dh2* (hdot*hdot) + dq_dh * hdotdot;
  res.qddd = d3q_dh3 * (hdot*hdot*hdot) +
             d2q_dh2 *hdot* hdotdot * 3.0 +
             dq_dh * hdotdotdot;

  // 将四元数的速度、加速度、加加速度转换成轴角的速度、加速度、加加速度
  convertQuaternionVelAccJerkToAngleVelAccJerk(
      res.q, res.qd, res.qdd, res.qddd, true, res.angle_vel_in_body_system,
      res.angle_acc_in_body_system, res.angle_jerk_in_body_system);
  res.rot = res.q.toRotationMatrix();
  res.angle_vel_in_space_system = rot * res.angle_vel_in_body_system;
  res.angle_acc_in_space_system = rot * res.angle_acc_in_body_system;
  res.angle_jerk_in_space_system = rot * res.angle_jerk_in_body_system;
  return res;
};

// 四元数绕固定轴旋转
QuaternionPoint QuaternionPoint::rotateAroundAxisByTheta(
    const Eigen::Vector3d axis, const double theta, bool is_body_system) {
  QuaternionPoint res;
  res.q = q.rotateAroundAxisByTheta(axis, theta, is_body_system);
  return res;
};
QuaternionPoint QuaternionPoint::rotateAroundAxisByTheta(
    const Eigen::Vector3d axis, const double theta, const double theta_dot,
    bool is_body_system) {

  QuaternionPoint res;
  res.q = q.rotateAroundAxisByTheta(axis, theta, is_body_system);
  res.rot = res.q.toRotationMatrix();
  if (is_body_system) {
    res.angle_vel_in_body_system = axis * theta_dot;
    res.angle_vel_in_space_system = res.rot * res.angle_vel_in_body_system;
  } else {
    res.angle_vel_in_space_system = axis * theta_dot;
    res.angle_vel_in_body_system =
        res.rot.transpose() * res.angle_vel_in_space_system;
  }

  return res;
};
QuaternionPoint QuaternionPoint ::rotateAroundAxisByTheta(
    const Eigen::Vector3d axis, const double theta, const double theta_dot,
    const double theta_dotdot, bool is_body_system) {

  QuaternionPoint res;
  res.q = q.rotateAroundAxisByTheta(axis, theta, is_body_system);
  res.rot = res.q.toRotationMatrix();
  if (is_body_system) {
    res.angle_vel_in_body_system = axis * theta_dot;
    res.angle_vel_in_space_system = res.rot * res.angle_vel_in_body_system;
    res.angle_acc_in_body_system = axis * theta_dotdot;
    res.angle_acc_in_space_system = res.rot * res.angle_acc_in_body_system;
  } else {
    res.angle_vel_in_space_system = axis * theta_dot;
    res.angle_vel_in_body_system =
        res.rot.transpose() * res.angle_vel_in_space_system;
    res.angle_acc_in_space_system = axis * theta_dotdot;
    res.angle_acc_in_body_system =
        res.rot.transpose() * res.angle_acc_in_space_system;
  }

  return res;
};
QuaternionPoint QuaternionPoint::rotateAroundAxisByTheta(
    const Eigen::Vector3d axis, const double theta, const double theta_dot,
    const double theta_dotdot, const double theta_dotdotdot,
    bool is_body_system) {
  QuaternionPoint res;
  res.q = q.rotateAroundAxisByTheta(axis, theta, is_body_system);
  res.rot = res.q.toRotationMatrix();
  if (is_body_system) {
    res.angle_vel_in_body_system = axis * theta_dot;
    res.angle_vel_in_space_system = res.rot * res.angle_vel_in_body_system;
    res.angle_acc_in_body_system = axis * theta_dotdot;
    res.angle_acc_in_space_system = res.rot * res.angle_acc_in_body_system;
    res.angle_jerk_in_body_system = axis * theta_dotdotdot;
    res.angle_jerk_in_space_system = res.rot * res.angle_jerk_in_body_system;
  } else {
    res.angle_vel_in_space_system = axis * theta_dot;
    res.angle_vel_in_body_system =
        res.rot.transpose() * res.angle_vel_in_space_system;
    res.angle_acc_in_space_system = axis * theta_dotdot;
    res.angle_acc_in_body_system =
        res.rot.transpose() * res.angle_acc_in_space_system;
    res.angle_jerk_in_space_system = axis * theta_dotdotdot;
    res.angle_jerk_in_body_system =
        res.rot.transpose() * res.angle_jerk_in_space_system;
  }

  return res;
};

Eigen::Vector3d QuaternionPoint::convertQuaternionVelToAngleVel(
    const Quaternion &quat, const Quaternion &quat_dot, bool is_body_system) {
  Eigen::Vector3d res;
  Quaternion tmp_angle_vel;
  if (is_body_system) {
    tmp_angle_vel = quat.inverse() * quat_dot * 2.0;
  } else {
    tmp_angle_vel = quat_dot * quat.inverse() * 2.0;
  }

  res << tmp_angle_vel.x, tmp_angle_vel.y, tmp_angle_vel.z;
  return res;
};
void QuaternionPoint::convertQuaternionVelAccToAngleVelAcc(
    const Quaternion &quat, const Quaternion &quat_dot,
    const Quaternion &quat_dotdot, bool is_body_system,
    Eigen::Vector3d &angle_vel, Eigen::Vector3d &angle_acc) {
  Quaternion tmp_angle_vel, tmp_angle_acc;
  if (is_body_system) {
    tmp_angle_vel = quat.inverse() * quat_dot * 2.0;
    tmp_angle_acc =
        quat.inverse() * (quat_dotdot * 2.0 - quat_dot * tmp_angle_vel);
  } else {
    tmp_angle_vel = quat_dot * quat.inverse() * 2.0;
    tmp_angle_acc =
        (quat_dotdot * 2.0 - tmp_angle_vel * quat_dot) * quat.inverse();
  }
  angle_vel << tmp_angle_vel.x, tmp_angle_vel.y, tmp_angle_vel.z;
  angle_acc << tmp_angle_acc.x, tmp_angle_acc.y, tmp_angle_acc.z;
};

void QuaternionPoint::convertQuaternionVelAccJerkToAngleVelAccJerk(
    const Quaternion &quat, const Quaternion &quat_dot,
    const Quaternion &quat_dotdot, const Quaternion &quat_dotdotdot,
    bool is_body_system, Eigen::Vector3d &angle_vel, Eigen::Vector3d &angle_acc,
    Eigen::Vector3d &angle_jerk) {
  Quaternion tmp_angle_vel, tmp_angle_acc, tmp_angle_jerk;
  if (is_body_system) {
    tmp_angle_vel = quat.inverse() * quat_dot * 2.0;
    tmp_angle_acc =
        quat.inverse() * (quat_dotdot * 2.0 - quat_dot * tmp_angle_vel);
    tmp_angle_jerk =
        quat.inverse() * (quat_dotdotdot * 2.0 - quat_dotdot * tmp_angle_vel -
                          quat_dot * tmp_angle_acc * 2.0);
  } else {
    tmp_angle_vel = quat_dot * quat.inverse() * 2.0;
    tmp_angle_acc =
        (quat_dotdot * 2.0 - tmp_angle_vel * quat_dot) * quat.inverse();
    tmp_angle_jerk = (quat_dotdotdot * 2.0 - tmp_angle_vel * quat_dotdot -
                      tmp_angle_acc * quat_dot * 2.0) *
                     quat.inverse();
  }
  angle_vel << tmp_angle_vel.x, tmp_angle_vel.y, tmp_angle_vel.z;
  angle_acc << tmp_angle_acc.x, tmp_angle_acc.y, tmp_angle_acc.z;
  angle_jerk << tmp_angle_jerk.x, tmp_angle_jerk.y, tmp_angle_jerk.z;
};
} // namespace planning
