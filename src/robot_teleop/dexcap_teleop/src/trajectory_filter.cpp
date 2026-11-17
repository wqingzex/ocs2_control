#include "dexcap_teleop/trajectory_filter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include <ruckig/ruckig.hpp>

namespace dexcap_teleop
{
namespace
{

constexpr double kEpsilon = 1.0e-9;

class FilterState : public TrajectoryFilter
{
public:
    explicit FilterState(const TrajectoryLimits &limits)
        : limits_(limits)
    {
        if (!std::isfinite(limits_.max_velocity) ||
            !std::isfinite(limits_.max_acceleration) ||
            !std::isfinite(limits_.max_jerk) ||
            !std::isfinite(limits_.tracking_frequency) ||
            !std::isfinite(limits_.target_filter_alpha) ||
            limits_.max_velocity <= 0.0 ||
            limits_.max_acceleration <= 0.0 ||
            limits_.max_jerk <= 0.0 ||
            limits_.tracking_frequency <= 0.0 ||
            limits_.target_filter_alpha <= 0.0 ||
            limits_.target_filter_alpha > 1.0)
        {
            throw std::invalid_argument("Invalid online trajectory limits");
        }
    }

    void reset(
        const double position, const double velocity,
        const double acceleration) override
    {
        position_ = std::isfinite(position) ? position : 0.0;
        velocity_ = std::clamp(
            std::isfinite(velocity) ? velocity : 0.0,
            -limits_.max_velocity, limits_.max_velocity);
        acceleration_ = std::clamp(
            std::isfinite(acceleration) ? acceleration : 0.0,
            -limits_.max_acceleration, limits_.max_acceleration);
        target_position_ = position_;
        requested_position_ = position_;
    }

    void set_target(const double position) override
    {
        if (std::isfinite(position))
        {
            requested_position_ = position;
        }
    }

protected:
    void filter_target()
    {
        target_position_ += limits_.target_filter_alpha *
            (requested_position_ - target_position_);
    }

    bool settled() const
    {
        return
            std::abs(requested_position_ - target_position_) <=
                limits_.position_tolerance &&
            std::abs(position_ - target_position_) <=
                limits_.position_tolerance &&
            std::abs(velocity_) <= limits_.velocity_tolerance &&
            std::abs(acceleration_) <= limits_.acceleration_tolerance;
    }

    TrajectoryLimits limits_;
    double position_{0.0};
    double velocity_{0.0};
    double acceleration_{0.0};
    double target_position_{0.0};
    double requested_position_{0.0};
};

class OnlineTrajectoryFilter final : public FilterState
{
public:
    using FilterState::FilterState;

    double update(const double dt) override
    {
        if (!std::isfinite(dt) || dt <= kEpsilon)
        {
            return position_;
        }
        filter_target();

        const double frequency = limits_.tracking_frequency;
        const double position_error = target_position_ - position_;
        const double desired_jerk =
            frequency * frequency * frequency * position_error -
            3.0 * frequency * frequency * velocity_ -
            3.0 * frequency * acceleration_;

        double lower_jerk = -limits_.max_jerk;
        double upper_jerk = limits_.max_jerk;
        lower_jerk = std::max(
            lower_jerk,
            (-limits_.max_acceleration - acceleration_) / dt);
        upper_jerk = std::min(
            upper_jerk,
            (limits_.max_acceleration - acceleration_) / dt);

        const double half_dt_squared = 0.5 * dt * dt;
        lower_jerk = std::max(
            lower_jerk,
            (-limits_.max_velocity - velocity_ - acceleration_ * dt) /
                half_dt_squared);
        upper_jerk = std::min(
            upper_jerk,
            (limits_.max_velocity - velocity_ - acceleration_ * dt) /
                half_dt_squared);

        const double jerk = lower_jerk <= upper_jerk
            ? std::clamp(desired_jerk, lower_jerk, upper_jerk)
            : std::clamp(
                -acceleration_ / dt,
                -limits_.max_jerk, limits_.max_jerk);
        const double next_acceleration = std::clamp(
            acceleration_ + dt * jerk,
            -limits_.max_acceleration, limits_.max_acceleration);
        const double next_velocity = std::clamp(
            velocity_ + dt * (acceleration_ + next_acceleration) / 2.0,
            -limits_.max_velocity, limits_.max_velocity);
        const double next_position =
            position_ + dt * (velocity_ + next_velocity) / 2.0;

        if (std::isfinite(next_position))
        {
            position_ = next_position;
            velocity_ = next_velocity;
            acceleration_ = next_acceleration;
        }
        if (settled())
        {
            position_ = requested_position_;
            velocity_ = 0.0;
            acceleration_ = 0.0;
        }
        return position_;
    }
};

class RuckigTrajectoryFilter final : public FilterState
{
public:
    using FilterState::FilterState;

    double update(const double dt) override
    {
        if (!std::isfinite(dt) || dt <= kEpsilon)
        {
            return position_;
        }
        filter_target();

        ruckig::Ruckig<1> trajectory(dt);
        ruckig::InputParameter<1> input;
        ruckig::OutputParameter<1> output;
        input.current_position = {position_};
        input.current_velocity = {velocity_};
        input.current_acceleration = {acceleration_};
        input.target_position = {target_position_};
        input.target_velocity = {0.0};
        input.target_acceleration = {0.0};
        input.max_velocity = {limits_.max_velocity};
        input.max_acceleration = {limits_.max_acceleration};
        input.max_jerk = {limits_.max_jerk};
        input.min_velocity =
            std::array<double, 1>{-limits_.max_velocity};
        input.min_acceleration =
            std::array<double, 1>{-limits_.max_acceleration};
        input.duration_discretization =
            ruckig::DurationDiscretization::Discrete;

        const auto result = trajectory.update(input, output);
        if (result < 0 ||
            !std::isfinite(output.new_position[0]) ||
            !std::isfinite(output.new_velocity[0]) ||
            !std::isfinite(output.new_acceleration[0]))
        {
            velocity_ = 0.0;
            acceleration_ = 0.0;
            return position_;
        }
        position_ = output.new_position[0];
        velocity_ = output.new_velocity[0];
        acceleration_ = output.new_acceleration[0];
        if (settled())
        {
            position_ = requested_position_;
            velocity_ = 0.0;
            acceleration_ = 0.0;
        }
        return position_;
    }
};

}  // namespace

std::unique_ptr<TrajectoryFilter> make_trajectory_filter(
    const std::string &type, const TrajectoryLimits &limits)
{
    if (type == "online")
    {
        return std::make_unique<OnlineTrajectoryFilter>(limits);
    }
    if (type == "ruckig")
    {
        return std::make_unique<RuckigTrajectoryFilter>(limits);
    }
    throw std::invalid_argument(
        "interpolation_type must be 'online' or 'ruckig'");
}

}  // namespace dexcap_teleop
