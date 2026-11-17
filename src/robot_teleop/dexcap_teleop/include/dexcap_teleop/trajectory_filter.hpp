#pragma once

#include <memory>
#include <string>

namespace dexcap_teleop
{

struct TrajectoryLimits
{
    double max_velocity{3.0};
    double max_acceleration{30.0};
    double max_jerk{300.0};
    double tracking_frequency{20.0};
    double target_filter_alpha{0.8};
    double position_tolerance{1.0e-4};
    double velocity_tolerance{1.0e-3};
    double acceleration_tolerance{1.0e-2};
};

class TrajectoryFilter
{
public:
    virtual ~TrajectoryFilter() = default;
    virtual void reset(
        double position, double velocity = 0.0,
        double acceleration = 0.0) = 0;
    virtual void set_target(double position) = 0;
    virtual double update(double dt) = 0;
};

std::unique_ptr<TrajectoryFilter> make_trajectory_filter(
    const std::string &type, const TrajectoryLimits &limits);

}  // namespace dexcap_teleop
