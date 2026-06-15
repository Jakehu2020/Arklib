#pragma once
#include "./PID.h"
#include "../hardware/odometry.h"
#include "./exits.h"
#include "./profiling.h"
#include "v5_vcs.h"
#include <functional>
#include <vector>
#include <unordered_map>
#include <string>
#include <memory>

using ControllerState = std::unordered_map<std::string, double>;
using ControllerGains = std::unordered_map<std::string, double>;
using MotionPath = std::vector<Velocity2d>;

class Chassis
{
private:
    volatile bool exit = false;
    std::shared_ptr<IExitCondition> defaultExit;

public:
    Ark2DMotorGroup &drivetrain;
    Odometry &odometry;
    double trackWidth;
    double wheelBase;
    Chassis(Ark2DMotorGroup &Drivetrain, Odometry &odometry, double trackWidth, double wheelBase);

    void tank(double left, double right);
    void move(double v, double omega);
    void regulatorControllerPath(
        std::function<Velocity2d(Chassis &, const Velocity2d &, ControllerState &, const ControllerGains &)> controller,
        const MotionPath &path,
        const ControllerGains &gains,
        double threshold = 1.0,
        std::shared_ptr<IExitCondition> exitOverride = nullptr);
    void trackingControllerPath(
        std::function<Velocity2d(Chassis &, const Velocity2d &, ControllerState &, const ControllerGains &, const MotionPath &)> controller,
        const MotionPath &path,
        const ControllerGains &gains,
        double threshold = 1.0,
        std::shared_ptr<IExitCondition> exitOverride = nullptr);
    void controlTarget(
        std::function<Velocity2d(Chassis &, const Velocity2d &, ControllerState &, const ControllerGains &)> controller,
        Pose2d &point,
        const ControllerGains &gains,
        double threshold,
        std::shared_ptr<IExitCondition> exitOverride = nullptr);
    void stop();

    void setDefaultExitCondition(std::shared_ptr<IExitCondition> exitCond);
};

// Defined after Chassis so Chassis is a complete type
using PointControllerFn = std::function<Velocity2d(Chassis &, const Velocity2d &, ControllerState &, const ControllerGains &)>;
using PathControllerFn = std::function<Velocity2d(Chassis &, const Velocity2d &, ControllerState &, const ControllerGains &, const MotionPath &)>;