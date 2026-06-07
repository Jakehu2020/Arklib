#include "../ark_custom/include/motion/controllers.h"
#include <functional>
#include <vector>
#include <unordered_map>
#include <string>

Velocity2d Kanayama(Chassis& chassis, const Velocity2d& target, ControllerState& state, const ControllerGains& gains) {
    Pose2d currentPose = chassis.odometry.getPose(true);
    Pose2d error = currentPose.relativeTo(target);

    double v = target.getLinearVelocity() * std::cos(error.getTheta()) + gains.at("kX") * error.getX();
    double omega = target.getAngularVelocity() + target.getLinearVelocity() * (gains.at("kY") * error.getY() + gains.at("kTheta") * std::sin(error.getTheta()));

    return Velocity2d(currentPose.getX(), currentPose.getY(), currentPose.getTheta(), v, omega, 0.0);
}