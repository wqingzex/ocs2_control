/******************************************************************************
 * OCS2 Wheel Humanoid - Base Lock Constraint
 * 
 * Constrains base velocity to zero when baseLocked mode is active.
 * This is a StateInputConstraint that is only active based on mode schedule.
 ******************************************************************************/

#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <whole_body_mpc/mode/SwitchedHumanoidReferenceManager.h>

namespace ocs2::whole_body_mpc {

/**
 * @brief Constraint to lock mobile base velocity to zero
 * 
 * When baseLocked mode is active, this equality constraint forces base
 * velocity inputs (linear and angular) to zero.
 */
class BaseLockConstraint : public StateInputConstraint {
public:
    /**
     * @param referenceManager Reference to the humanoid reference manager
     * @param baseInputDim Dimension of base inputs (typically 2: linear, angular)
     */
    BaseLockConstraint(const SwitchedHumanoidReferenceManager& referenceManager,
                       size_t baseInputDim)
        : StateInputConstraint(ConstraintOrder::Linear),
          referenceManagerPtr_(&referenceManager),
          baseInputDim_(baseInputDim) {}

    ~BaseLockConstraint() override = default;
    BaseLockConstraint* clone() const override { return new BaseLockConstraint(*this); }

    size_t getNumConstraints(scalar_t time) const override {
        // Only active when base is locked
        return referenceManagerPtr_->isBaseLocked(time) ? baseInputDim_ : 0;
    }

    vector_t getValue(scalar_t time, const vector_t& state, 
                      const vector_t& input, const PreComputation&) const override {
        if (!referenceManagerPtr_->isBaseLocked(time)) {
            return vector_t();  // No constraint
        }
        
        // Constrain base inputs (first baseInputDim_ inputs) to zero
        return input.head(baseInputDim_);
    }

    VectorFunctionLinearApproximation getLinearApproximation(
        scalar_t time, const vector_t& state, const vector_t& input,
        const PreComputation&) const override {
        
        if (!referenceManagerPtr_->isBaseLocked(time)) {
            return VectorFunctionLinearApproximation::Zero(0, state.size(), input.size());
        }
        
        VectorFunctionLinearApproximation approx(baseInputDim_, state.size(), input.size());
        approx.f = input.head(baseInputDim_);
        approx.dfdx.setZero();
        approx.dfdu.setZero();
        approx.dfdu.leftCols(baseInputDim_).setIdentity();
        return approx;
    }

private:
    const SwitchedHumanoidReferenceManager* referenceManagerPtr_;
    size_t baseInputDim_;
};

}  // namespace ocs2::whole_body_mpc

