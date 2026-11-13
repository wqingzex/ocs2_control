//
// What PoseBasedReferenceManager needs to know about bimanual coupling.
//
// The whole-body MPC owns the coupling state (which mode is active, the captured relative pose). The reference
// manager only asks; it looks for this interface on the OCS2 reference manager it wraps (dynamic_cast), so the
// generic runtime does not depend on any robot-specific MPC package. The whole-body reference manager implements it.
//
#pragma once

#include <utility>

#include <Eigen/Geometry>
#include <ocs2_core/Types.h>

namespace ocs2::mpc_core
{
    class BimanualCouplingSource
    {
    public:
        virtual ~BimanualCouplingSource() = default;

        /** True while both arms are driven as one coupled body at `time`. */
        [[nodiscard]] virtual bool isBimanualCoupled(scalar_t time) const = 0;
        /** True once the relative pose between the arms was captured (otherwise the default offset is used). */
        [[nodiscard]] virtual bool hasCapturedCoupling() const = 0;
        /** Right-arm offset in the left end-effector frame: position and orientation. */
        [[nodiscard]] virtual std::pair<Eigen::Matrix<scalar_t, 3, 1>, Eigen::Quaternion<scalar_t>> getCouplingParameters(
            scalar_t time) const = 0;
    };
} // namespace ocs2::mpc_core
