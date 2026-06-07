#include "../ark_custom/include/motion/controllers.h"
#include <functional>
#include <vector>
#include <unordered_map>
#include <string>

// EXAMPLE LQR GAINS - CALCULATE THESE BEFOREHAND
//     K row 0: v = -K[0] * [ex, ey, etheta]
//     K row 1: omega = -K[1] * [ex, ey, etheta]

// const ControllerGains lqrGains = {
//     {"k00", 3.162}, {"k01", 0.0}, {"k02", 1.847},
//     {"k10", 0.0}, {"k11", 3.162}, {"k12", 2.314},
// };

Velocity2d LTV_LQR(Chassis &chassis, const Velocity2d &target, ControllerState &state, const ControllerGains &gains)
{
    Pose2d current = chassis.odometry.getPose(true);

    Pose2d err = current.relativeTo(target);

    double dv = -(gains.at("k00") * err.getX() + gains.at("k01") * err.getY() + gains.at("k02") * err.getTheta());
    double domega = -(gains.at("k10") * err.getX() + gains.at("k11") * err.getY() + gains.at("k12") * err.getTheta());

    return Velocity2d(current, target.getLinearVelocity() + dv, target.getAngularVelocity() + domega, 0.0);
}