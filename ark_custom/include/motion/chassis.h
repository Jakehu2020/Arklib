#pragma once
#include "./PID.h"
#include "../hardware/odometry.h"
#include "./profiling.h"
#include "v5_vcs.h"
#include <functional>
#include <vector>
#include <unordered_map>
#include <string>

using ControllerState = std::unordered_map<std::string, double>;
using ControllerGains = std::unordered_map<std::string, double>;
using MotionPath      = std::vector<Velocity2d>;

class Chassis {
    private:
        volatile bool exit = false;
    public:
        Ark2DMotorGroup& drivetrain;
        Odometry& odometry;
        double trackWidth;
        double wheelBase;
        Chassis(Ark2DMotorGroup& Drivetrain, Odometry& odometry, double trackWidth, double wheelBase);

        void tank(double left, double right);
        void move(double v, double omega);
        void controlPath(
            std::function<Velocity2d(Chassis&, const Velocity2d&, ControllerState&, const ControllerGains&)> controller,
            const MotionPath& path,
            const ControllerGains& gains,
            double threshold = 1.0
        );
        void controlTarget(
            std::function<Velocity2d(Chassis&, const Velocity2d&, ControllerState&, const ControllerGains&)> controller,
            Pose2d& point,
            const ControllerGains& gains,
            double threshold
        );
        void stop();
        bool atTarget(Velocity2d target, double threshold);
};

// Defined after Chassis so Chassis is a complete type
using ControllerFn = std::function<Velocity2d(Chassis&, const Velocity2d&, ControllerState&, const ControllerGains&)>;