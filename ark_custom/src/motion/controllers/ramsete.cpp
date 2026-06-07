#include "../ark_custom/include/motion/controllers.h"
#include <functional>
#include <vector>
#include <unordered_map>
#include <string>

Velocity2d Ramsete(Chassis& chassis, const Velocity2d& target, ControllerState& state, const ControllerGains& gains) {
    Pose2d currentPose = chassis.odometry.getPose(true);
    Pose2d error = currentPose.relativeTo(target);

    double k = 2.0 * gains.at("zeta") * std::sqrt(target.getAngularVelocity() * target.getAngularVelocity() + gains.at("b") * target.getLinearVelocity() * target.getLinearVelocity());

    double v = target.getLinearVelocity() * std::cos(error.getTheta()) + k * error.getX();
    double omega = target.getAngularVelocity() + k * error.getTheta() + gains.at("b") * target.getLinearVelocity() * std::sin(error.getTheta()) * error.getY() / error.getTheta();

    return Velocity2d(currentPose.getX(), currentPose.getY(), currentPose.getTheta(), v, omega, 0.0);
}