//
// PoseBasedReferenceManager: wheel-humanoid bimanual coupling (right marker follows the left)
//

#include "mpc_core/reference/PoseBasedReferenceManager.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <fstream>
#include <iomanip>
#include <limits>

#include "mpc_core/reference/BimanualCouplingSource.h"

namespace ocs2::mpc_core
{

    void PoseBasedReferenceManager::setCoupledRightMarkerFromLeftPose(const vector_t& left_pose7_xyzw, bool publish)
    {
        if (!dual_arm_mode_ || left_pose7_xyzw.size() < 7) {
            return;
        }
        auto* sw = dynamic_cast<const BimanualCouplingSource*>(referenceManagerPtr_.get());
        if (sw == nullptr || !sw->hasCapturedCoupling()) {
            return;
        }
        const scalar_t t = current_observation_.time;
        if (!sw->isBimanualCoupled(t)) {
            return;
        }
        const auto [relPos, relQuatRaw] = sw->getCouplingParameters(t);
        Eigen::Quaternion<scalar_t> relQuat = relQuatRaw;
        if (relQuat.norm() < 1e-9) {
            return;
        }
        relQuat.normalize();

        Eigen::Quaternion leftQuat(left_pose7_xyzw(6), left_pose7_xyzw(3), left_pose7_xyzw(4),
                                             left_pose7_xyzw(5));
        if (leftQuat.norm() < 1e-9) {
            return;
        }
        leftQuat.normalize();

        if (!armExternalCommandAllowed(false))
        {
            return;
        }
        const Eigen::Matrix<scalar_t, 3, 1> rightPos = left_pose7_xyzw.head<3>() + leftQuat * relPos;
        const Eigen::Quaternion<scalar_t> rightQuat = (leftQuat * relQuat).normalized();
        right_target_state_.head<3>() = rightPos;
        right_target_state_(3) = rightQuat.x();
        right_target_state_(4) = rightQuat.y();
        right_target_state_(5) = rightQuat.z();
        right_target_state_(6) = rightQuat.w();
        if (publish) {
            publishCurrentTargets("right");
        }
    }

    bool PoseBasedReferenceManager::syncWholeBodyCoupledOppositeArmIfNeeded(bool left_target_was_updated)
    {
        if (!isWholeBodyTargetLayout()) {
            return false;
        }
        auto* sw = dynamic_cast<const BimanualCouplingSource*>(referenceManagerPtr_.get());
        if (sw == nullptr) {
            return false;
        }
        const scalar_t t = current_observation_.time;
        if (!sw->isBimanualCoupled(t) || !sw->hasCapturedCoupling()) {
            return false;
        }
        const auto rel = sw->getCouplingParameters(t);
        const Eigen::Matrix<scalar_t, 3, 1> relPos = rel.first;
        Eigen::Quaternion<scalar_t> relQuat = rel.second;
        if (relQuat.norm() < static_cast<scalar_t>(1e-9)) {
            return false;
        }
        relQuat.normalize();

        if (left_target_was_updated) {
            if (!armExternalCommandAllowed(false))
            {
                return false;
            }
            const vector_t& left = left_target_state_;
            vector_t& right = right_target_state_;
            const Eigen::Matrix<scalar_t, 3, 1> leftPos = left.head<3>();
            Eigen::Quaternion<scalar_t> leftQuat(left(6), left(3), left(4), left(5));
            leftQuat.normalize();
            const Eigen::Matrix<scalar_t, 3, 1> rightPos = leftPos + leftQuat * relPos;
            Eigen::Quaternion<scalar_t> rightQuat = (leftQuat * relQuat).normalized();
            right.head<3>() = rightPos;
            right(3) = rightQuat.x();
            right(4) = rightQuat.y();
            right(5) = rightQuat.z();
            right(6) = rightQuat.w();
        } else {
            if (!armExternalCommandAllowed(true))
            {
                return false;
            }
            vector_t& left = left_target_state_;
            const vector_t& right = right_target_state_;
            const Eigen::Matrix<scalar_t, 3, 1> rightPos = right.head<3>();
            Eigen::Quaternion<scalar_t> rightQuat(right(6), right(3), right(4), right(5));
            rightQuat.normalize();
            const Eigen::Quaternion<scalar_t> invRelQuat = relQuat.inverse();
            const Eigen::Matrix<scalar_t, 3, 1> invRelPos = -(invRelQuat * relPos);
            const Eigen::Quaternion<scalar_t> leftQuat = (rightQuat * invRelQuat).normalized();
            const Eigen::Matrix<scalar_t, 3, 1> leftPos = rightPos + rightQuat * invRelPos;
            left.head<3>() = leftPos;
            left(3) = leftQuat.x();
            left(4) = leftQuat.y();
            left(5) = leftQuat.z();
            left(6) = leftQuat.w();
        }
        return true;
    }

    bool PoseBasedReferenceManager::isWholeBodyBimanualCoupled() const
    {
        if (!isWholeBodyTargetLayout())
        {
            return false;
        }
        auto* sw = dynamic_cast<const BimanualCouplingSource*>(referenceManagerPtr_.get());
        if (sw == nullptr)
        {
            return false;
        }
        return sw->isBimanualCoupled(current_observation_.time) && sw->hasCapturedCoupling();
    }

    void PoseBasedReferenceManager::markLeftCouplingCommand()
    {
        last_left_coupling_command_stamp_ = std::chrono::steady_clock::now();
    }

    bool PoseBasedReferenceManager::isCoupledLeftLeaderFresh() const
    {
        if (last_left_coupling_command_stamp_.time_since_epoch().count() == 0)
        {
            return false;
        }
        const double age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_left_coupling_command_stamp_).count();
        return age < kCoupledLeftLeaderTimeoutSec_;
    }

} // namespace ocs2::mpc_core
