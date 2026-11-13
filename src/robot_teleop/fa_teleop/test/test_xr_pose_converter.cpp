#include <gtest/gtest.h>

#include <cmath>

#include "fa_teleop/xr_pose_converter.hpp"
#include "golden_csv.hpp"

using fa_teleop::Pose7;
using fa_teleop::XrFrame;
using fa_teleop::XrPoseConverter;

namespace
{
    Pose7 slice(const std::vector<double>& row, size_t offset)
    {
        Pose7 p{};
        for (size_t k = 0; k < 7; ++k)
        {
            p[k] = row[offset + k];
        }
        return p;
    }

    /// Positions must match; quaternions match up to sign (q and -q are the same rotation).
    void expectSamePose(const Pose7& actual, const Pose7& expected, const std::string& what)
    {
        for (size_t k = 0; k < 3; ++k)
        {
            EXPECT_NEAR(actual[k], expected[k], 1e-9) << what << " position " << k;
        }
        double dot = 0.0;
        for (size_t k = 3; k < 7; ++k)
        {
            dot += actual[k] * expected[k];
        }
        EXPECT_NEAR(std::abs(dot), 1.0, 1e-9) << what << " orientation";
    }
} // namespace

TEST(XrPoseConverterGolden, matchesPythonReferenceIncludingFallbacks)
{
    const auto rows = fa_teleop::test::readCsv(fa_teleop::test::dataPath("xr_poses_golden.csv"));
    ASSERT_GT(rows.size(), 100u);

    XrPoseConverter converter;
    for (size_t i = 0; i < rows.size(); ++i)
    {
        const auto& r = rows[i];
        ASSERT_EQ(r.size(), 42u) << "row " << i;
        XrFrame frame;
        frame.head = slice(r, 0);
        frame.left.pose = slice(r, 7);
        frame.right.pose = slice(r, 14);
        frame.has_head = frame.has_left = frame.has_right = true;
        const auto& poses = converter.update(frame);
        const std::string tag = "frame " + std::to_string(i);
        expectSamePose(XrPoseConverter::toPose7(poses.head), slice(r, 21), tag + " head");
        expectSamePose(XrPoseConverter::toPose7(poses.left), slice(r, 28), tag + " left");
        expectSamePose(XrPoseConverter::toPose7(poses.right), slice(r, 35), tag + " right");
        if (::testing::Test::HasFailure())
        {
            return;  // first mismatch is enough
        }
    }
}

TEST(XrPoseConverter, openXrForwardMapsToRobotForward)
{
    // OpenXR: -Z is forward, +Y up, +X right. Robot: +X forward, +Z up, +Y left.
    const auto pose = XrPoseConverter::openXrToRobot({0.0, 1.0, -2.0, 0, 0, 0, 1});
    ASSERT_TRUE(pose.has_value());
    EXPECT_NEAR(pose->translation().x(), 2.0, 1e-12);
    EXPECT_NEAR(pose->translation().y(), 0.0, 1e-12);
    EXPECT_NEAR(pose->translation().z(), 1.0, 1e-12);
}

TEST(XrPoseConverter, invalidSampleKeepsPreviousPose)
{
    XrPoseConverter converter;
    XrFrame frame;
    frame.has_head = true;
    frame.head = {1.0, 0.0, 0.0, 0, 0, 0, 1};
    const Eigen::Vector3d first = converter.update(frame).head.translation();

    frame.head = {NAN, 0, 0, 0, 0, 0, 1};
    EXPECT_TRUE(converter.update(frame).head.translation().isApprox(first));
    frame.head = {5.0, 5.0, 5.0, 0, 0, 0, 0};  // zero quaternion
    EXPECT_TRUE(converter.update(frame).head.translation().isApprox(first));
}

TEST(XrPoseConverter, missingDeviceKeepsInitialReferencePose)
{
    XrPoseConverter converter;
    XrFrame frame;  // has_* = false
    const auto& poses = converter.update(frame);
    EXPECT_NEAR(poses.head.translation().y(), 1.5, 1e-12);
    EXPECT_NEAR(poses.left.translation().x(), -0.15, 1e-12);
}
