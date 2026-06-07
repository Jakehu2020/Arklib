#include "../ark_custom/include/motion/chassis.h"
#include "v5_vcs.h"
#include <functional>
#include <vector>
#include <array>
#include <unordered_map>
#include <string>

Chassis::Chassis(Ark2DMotorGroup& Drivetrain, Odometry& odometry, double trackWidth, double wheelBase)
    : drivetrain(Drivetrain), odometry(odometry), trackWidth(trackWidth), wheelBase(wheelBase){};

void Chassis::tank(double left, double right){
    drivetrain.move({ (left + right)/2, (left - right)/2 });
}
void Chassis::move(double v, double omega){
    // This should be modified respective to the actual configuration of the drivetrain. The current implementation is for a
    // default differential drivetrain. Modifications can be made for holonomic/mecanum, x-drive, swerve, etc.
    tank(v - omega * trackWidth / 2.0, v + omega * trackWidth / 2.0);
}

void Chassis::controlPath(
            ControllerFn controller,
            const MotionPath& path,
            const ControllerGains& gains,
            double threshold
        ) {
    exit = false;
    int step = 0;
    std::unordered_map<std::string, double> state;
    while (!exit) {
        if(step >= path.size()) {
            break;
        }
        Velocity2d target = controller(*this, path[step], state, gains);
        move(target.getLinearVelocity(), target.getAngularVelocity());

        if (atTarget(target, threshold)) {
            step++;
        }
        
        vex::task::sleep(20);
    }
    stop();
}

void Chassis::controlTarget(
            ControllerFn controller,
            Pose2d& point,
            const ControllerGains& gains,
            double threshold
        ) {
    exit = false;
    MotionPath path = MotionProfile::bezierSpline(
        {{odometry.getPose(true), point}},
        gains.at("vMax"),
        gains.at("curvatureGain"), 20, 0.1);
    controlPath(controller, path, gains, threshold);
    
    stop();
}

void Chassis::stop() {
    exit = true;
}

bool Chassis::atTarget(Velocity2d target, double threshold) {
    Pose2d currentPose = odometry.getPose(true);
    double distance = currentPose.getDistance(target);
    return distance < threshold;
}

/*

Logistics:
1. Have a separate exit-condition system, custom to the user
    a) Big-exit Small-exit error (as done by EZ)
    b) End at a radius from the target
    c) (x, y, theta) are all within their respective thresholds
2. Set the drivetrain to do one specific motion-control at a time, which can be stopped either:
    a) Manually, by user defintion chassis.stop() or an alternative
    b) Set at a braking point when the chassis is within a radius of a point (of course, with a break)
    c) Automatically when within range of the final target.
3. Motion controls should be individual functions with parameters:
    a) Chassis: The chassis object that is being controlled (for current position, etc)
    b) Target: A 2D motion profile through a Bezier/Quintic Hermite Spline
std::vector<std::array<double, 5>> target;
// [0] x, [1] y, [2] theta, [3] v, [4] omega
        i) The motion controller will iterate through these when in a certain range from the individual target points on the 2DMP.
    c) State: Previous information that is relevant to storing
std::unordered_map<std::string, std::variant<int, float, std::string>> state;
    d) Data: Relevant tuning gains/parameters
std::unordered_map<std::string, float> data;
    e) Default values should be applicable so that the user shouldn't have to write the same gains multiple times.
4. Motion Profiles are also individual functions that:
    a) Input a select amount of endpoints that define the general shape of the profile
    b) Output a more detailed vector of points
    c) Types of profiles should include:
        i) Bezier-endpoint Splines
        ii) c^2-continuous Quintic Hermite Splines
        iii) point-to-point Linear lines
    d) The vector of points can be a minimum distance from each other.
*/