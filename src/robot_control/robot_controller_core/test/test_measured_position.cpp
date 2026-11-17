// A measured joint position that cannot be read must never turn into 0.0 (the robot would be driven there).
#include <gtest/gtest.h>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <memory>
#include <optional>

#include "robot_controller_core/CtrlInterfaces.h"

using robot_controller_core::CtrlInterfaces;
using robot_controller_core::resolveMeasuredPosition;

TEST(ResolveMeasuredPosition, returns_the_reading_and_remembers_it)
{
    std::atomic<double> last_good{0.0};
    std::atomic<std::size_t> failures{0};

    EXPECT_DOUBLE_EQ(resolveMeasuredPosition(std::optional<double>{1.25}, last_good, failures), 1.25);
    EXPECT_DOUBLE_EQ(last_good.load(), 1.25);
    EXPECT_EQ(failures.load(), 0u);
}

TEST(ResolveMeasuredPosition, a_failed_read_returns_the_last_good_value_not_zero)
{
    std::atomic<double> last_good{0.0};
    std::atomic<std::size_t> failures{0};
    resolveMeasuredPosition(std::optional<double>{-2.6}, last_good, failures);

    EXPECT_DOUBLE_EQ(resolveMeasuredPosition(std::nullopt, last_good, failures), -2.6);
    EXPECT_DOUBLE_EQ(resolveMeasuredPosition(std::nullopt, last_good, failures), -2.6);
    EXPECT_EQ(failures.load(), 2u);
    // A later successful read replaces the remembered value.
    EXPECT_DOUBLE_EQ(resolveMeasuredPosition(std::optional<double>{0.3}, last_good, failures), 0.3);
    EXPECT_DOUBLE_EQ(resolveMeasuredPosition(std::nullopt, last_good, failures), 0.3);
}

namespace
{
// Two joints backed by plain doubles, wired the way the controller receives them.
struct Fixture
{
    double position[2]{0.5, -1.5};
    std::shared_ptr<hardware_interface::StateInterface> state[2];
    std::unique_ptr<hardware_interface::LoanedStateInterface> loaned[2];
    CtrlInterfaces interfaces;

    Fixture()
    {
        for (int i = 0; i < 2; ++i)
        {
            state[i] = std::make_shared<hardware_interface::StateInterface>("joint" + std::to_string(i), "position",
                                                                             &position[i]);
            loaned[i] = std::make_unique<hardware_interface::LoanedStateInterface>(state[i]);
            interfaces.joint_position_state_interface_.push_back(*loaned[i]);
        }
    }
};
}  // namespace

TEST(CtrlInterfacesPosition, starts_from_the_measured_positions_and_follows_them)
{
    Fixture f;
    f.interfaces.initializeLastSentPositions();

    EXPECT_DOUBLE_EQ(f.interfaces.last_sent_joint_positions_[0], 0.5);
    EXPECT_DOUBLE_EQ(f.interfaces.last_sent_joint_positions_[1], -1.5);
    f.position[1] = 2.0;
    EXPECT_DOUBLE_EQ(f.interfaces.measuredJointPosition(1), 2.0);
    EXPECT_EQ(f.interfaces.takePositionReadFailures(), 0u);
}

TEST(CtrlInterfacesPosition, the_commanded_position_is_the_last_one_sent_not_a_default)
{
    Fixture f;
    f.interfaces.initializeLastSentPositions();
    f.interfaces.last_sent_joint_positions_[0] = 0.9;
    EXPECT_DOUBLE_EQ(f.interfaces.commandedJointPosition(0), 0.9);
}

TEST(CtrlInterfacesPosition, reading_before_initialization_or_out_of_range_is_an_error)
{
    Fixture f;
    EXPECT_THROW(f.interfaces.measuredJointPosition(0), std::logic_error);
    f.interfaces.initializeLastSentPositions();
    EXPECT_THROW(f.interfaces.measuredJointPosition(2), std::logic_error);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
