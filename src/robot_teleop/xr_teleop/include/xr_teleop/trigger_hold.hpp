#pragma once

namespace xr_teleop
{
    /// The trigger value to publish for one controller. A controller that is not in this frame (switched off, tracking
    /// lost) has a default sample with trigger 0, which downstream means "gripper fully open": an object in the hand
    /// would be dropped. While the controller is absent the last value it had is kept.
    inline double triggerOrHeld(bool controller_present, double value, double& held)
    {
        if (controller_present) { held = value; }
        return held;
    }
}  // namespace xr_teleop
