#include "../ark_custom/include/motion/chassis.h"
#pragma once

namespace controller {

struct DriveParams {
    /// Deadband threshold — inputs below this magnitude are treated as zero.
    double deadband            = 0.01;

    /// Slew rate limit for throttle changes (per iteration).
    double slew                = 0.02;

    /// Turn nonlinearity factor (0–1). Higher values make the turn curve
    /// more aggressive near the extremes.
    double turnNonlinearity    = 0.5;

    /// Sensitivity multiplier for angular command when moving linearly.
    double sensitivity         = 1.0;

    /// Scalar applied to the change in turn input for the negative inertia
    /// accumulator.
    double negInertiaScalar    = 5.0;
};

} // namespace controller
