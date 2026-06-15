#include "./chassis.h"
#include "v5_vcs.h"
#include <functional>
#include <vector>
#include <array>
#include <unordered_map>
#include <string>

using ControllerState = std::unordered_map<std::string, double>;
using ControllerGains = std::unordered_map<std::string, double>;
using MotionPath = std::vector<Velocity2d>;

// Point Controllers
Velocity2d Kanayama(Chassis& chassis, const Velocity2d& target, ControllerState& state, const ControllerGains& gains);
Velocity2d RAMSETE(Chassis& chassis, const Velocity2d& target, ControllerState& state, const ControllerGains& gains);
Velocity2d LTI_LQR(Chassis& chassis, const Velocity2d& target, ControllerState& state, const ControllerGains& gains);

// Path Controllers
Velocity2d LTV_LQR(Chassis& chassis, const Velocity2d& target, ControllerState& state, const ControllerGains& gains, const MotionPath& path);
Velocity2d MPC(Chassis& chassis, const Velocity2d& target, ControllerState& state, const ControllerGains& gains, const MotionPath& path);
