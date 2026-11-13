#include "motion_planning/velocity_planner/plan_velocity_using_doubles.h"
#include "motion_planning/common/logger.h"
#include <math.h>
#include <rclcpp/logging.hpp>

namespace planning {

bool PlanVelocityUsingDoubles::init(double p0, double v0, double v1,
                                    double _max_acc, double _max_jerk,
                                    double _period, double _totaltime) {
  setZeroPara();
  start_pos = p0;
  start_vel = v0;
  end_vel = v1;
  period = fabs(_period);
  bool is_acc = v1 - v0 > min_val ? true : false;
  double taa, tua, acc_totaltime;
  if (is_acc) { // 加速
    if (!calculateAccelerationParameter(v0, v1, _max_acc, _max_jerk, &taa,
                                        &tua)) {
      return false;
    }

  } else { // 减速
    if (!calculateAccelerationParameter(-v0, -v1, _max_acc, _max_jerk, &taa,
                                        &tua)) {
      return false;
    }
  }
  acc_totaltime = taa * 2.0 + tua;
  if (!is_acc) { // 减速就忽略用户设置的时间
    tj2 = taa;
    td = acc_totaltime;
    totaltime = td;
  } else {
    if (_totaltime > min_val) { // 用户设置了运行时间
      if (_totaltime >
          acc_totaltime) { // 用户设置的运行时间大于加速到目标速度的时间
        tj1 = taa;
        ta = acc_totaltime;
        tv = _totaltime - ta;
        totaltime = _totaltime;
      } else { // 否则会重新计算目标速度
        if (taa < 0.5 * _totaltime) {
          tj1 = taa;
        } else {
          tj1 = 0.5 * _totaltime;
        }
        ta = _totaltime;
        end_vel = (ta - tj1) * tj1 * max_jerk + start_vel;
        totaltime = _totaltime;
      }
    } else { // 用户没有设置运行时间，就没有匀速段，只是加速到目标速度
      tj1 = taa;
      ta = acc_totaltime;
      totaltime = ta;
    }
  }
  calculateDistanceOf7thPhase();
  end_pos = start_pos + distance_acc + distance_v + distance_dec;
  return true;
}

bool PlanVelocityUsingDoubles::calculateAccelerationParameter(
    double v0, double v1, double _max_acc, double _max_jerk, double *taa,
    double *tua) {

  max_acc = abs(_max_acc);
  max_jerk = abs(_max_jerk);
  double delta_vel = v1 - v0;
  if (delta_vel < min_val) {

    RCLCPP_WARN_STREAM(planning::logger(), "Current velocity is equal to or greater than the target "
                 "velocity,no acceleration will be executed.");
    return false;
  }
  if (max_acc < min_val) {
    RCLCPP_ERROR_STREAM(planning::logger(), "Max acc is less than or equal to 0,Please input a positive "
                 "parameter of max acc");
  }

  if (max_jerk < min_val) {
    RCLCPP_ERROR_STREAM(planning::logger(), "Max jerk is less than or equal to 0,Please input a positive "
                 "parameter of max jerk");
  }

  if (max_acc * max_acc / max_jerk - delta_vel <- min_val) { // 有匀加速段
    *taa = max_acc / max_jerk;
    *tua = (delta_vel - *taa * max_acc) / max_acc;
  } else {
    *taa = sqrt(delta_vel / max_jerk);
    *tua = 0.0;
  }
  return true;
}

void PlanVelocityUsingDoubles::setZeroPara() {
  tj1 = 0.0;
  tj2 = 0.0;
  ta = 0.0;
  td = 0.0;
  tv = 0.0;
  totaltime = 0.0;
}

} // namespace planning
