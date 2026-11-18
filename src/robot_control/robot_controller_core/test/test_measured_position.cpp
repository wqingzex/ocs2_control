// A measured joint position that cannot be read must never turn into 0.0 (the robot would be driven there).
#include <gtest/gtest.h>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <limits>
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

namespace
{
// The command side of the same two joints: the exit setJointPositionCommand writes into these handles.
struct CommandFixture : Fixture
{
    std::shared_ptr<hardware_interface::CommandInterface> command[2];
    std::unique_ptr<hardware_interface::LoanedCommandInterface> loaned_command[2];
    std::shared_ptr<hardware_interface::CommandInterface> velocity, force;  // joint0 feed-forward of the MIX mode
    std::unique_ptr<hardware_interface::LoanedCommandInterface> loaned_velocity, loaned_force;

    static std::shared_ptr<hardware_interface::CommandInterface> make(const std::string& name)
    {
        hardware_interface::InterfaceInfo info;
        info.name = name;
        return std::make_shared<hardware_interface::CommandInterface>(
            hardware_interface::InterfaceDescription("joint0", info));
    }

    CommandFixture()
    {
        velocity = make("velocity");
        force = make("effort");
        loaned_velocity = std::make_unique<hardware_interface::LoanedCommandInterface>(velocity, nullptr);
        loaned_force = std::make_unique<hardware_interface::LoanedCommandInterface>(force, nullptr);
        interfaces.joint_velocity_command_interface_.push_back(*loaned_velocity);
        interfaces.joint_force_command_interface_.push_back(*loaned_force);
        for (int i = 0; i < 2; ++i)
        {
            hardware_interface::InterfaceInfo info;
            info.name = "position";
            command[i] = std::make_shared<hardware_interface::CommandInterface>(
                hardware_interface::InterfaceDescription("joint" + std::to_string(i), info));
            loaned_command[i] = std::make_unique<hardware_interface::LoanedCommandInterface>(command[i], nullptr);
            interfaces.joint_position_command_interface_.push_back(*loaned_command[i]);
        }
        interfaces.initializeLastSentPositions();
    }
};
}  // namespace

TEST(CtrlInterfacesCommand, a_finite_command_is_written_and_remembered)
{
    CommandFixture f;
    f.interfaces.setJointPositionCommand(0, 0.75);

    EXPECT_DOUBLE_EQ(f.interfaces.commandedJointPosition(0), 0.75);
    EXPECT_DOUBLE_EQ(*f.loaned_command[0]->get_optional(), 0.75);
    EXPECT_EQ(f.interfaces.takeRejectedCommands(), 0u);
}

TEST(CtrlInterfacesCommand, nan_and_inf_are_refused_and_the_joint_keeps_its_last_command)
{
    CommandFixture f;
    f.interfaces.setJointPositionCommand(1, 0.4);

    f.interfaces.setJointPositionCommand(1, std::numeric_limits<double>::quiet_NaN());
    f.interfaces.setJointPositionCommand(1, std::numeric_limits<double>::infinity());

    EXPECT_DOUBLE_EQ(f.interfaces.commandedJointPosition(1), 0.4);
    EXPECT_DOUBLE_EQ(*f.loaned_command[1]->get_optional(), 0.4);
    EXPECT_EQ(f.interfaces.takeRejectedCommands(), 2u);
    EXPECT_EQ(f.interfaces.takeRejectedCommands(), 0u);  // taking resets the count
}

TEST(CtrlInterfacesCommand, a_good_command_after_a_refused_one_goes_through)
{
    CommandFixture f;
    f.interfaces.setJointPositionCommand(0, std::numeric_limits<double>::quiet_NaN());
    f.interfaces.setJointPositionCommand(0, -0.2);

    EXPECT_DOUBLE_EQ(f.interfaces.commandedJointPosition(0), -0.2);
    EXPECT_EQ(f.interfaces.takeRejectedCommands(), 1u);
}

TEST(CtrlInterfacesCommand, velocity_and_torque_feed_forward_go_through_the_same_check)
{
    CommandFixture f;
    f.interfaces.setJointVelocityCommand(0, 0.5);
    f.interfaces.setJointForceCommand(0, 3.0);
    EXPECT_DOUBLE_EQ(*f.loaned_velocity->get_optional(), 0.5);
    EXPECT_DOUBLE_EQ(*f.loaned_force->get_optional(), 3.0);
    EXPECT_EQ(f.interfaces.takeRejectedCommands(), 0u);

    f.interfaces.setJointVelocityCommand(0, std::numeric_limits<double>::quiet_NaN());
    f.interfaces.setJointForceCommand(0, std::numeric_limits<double>::infinity());
    EXPECT_DOUBLE_EQ(*f.loaned_velocity->get_optional(), 0.5) << "the drive keeps the last velocity";
    EXPECT_DOUBLE_EQ(*f.loaned_force->get_optional(), 3.0) << "the drive keeps the last torque";
    EXPECT_EQ(f.interfaces.takeRejectedCommands(), 2u);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
