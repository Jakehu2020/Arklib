#include "./chassis.h"
#include "v5_vcs.h"
#include <functional>
#include <vector>
#include <array>
#include <unordered_map>
#include <string>

using ControllerState = std::unordered_map<std::string, double>;
using ControllerGains = std::unordered_map<std::string, double>;
using MotionPath      = std::vector<Velocity2d>;

Velocity2d Kanayama(Chassis& chassis, const Velocity2d& target, ControllerState& state, const ControllerGains& gains);
Velocity2d RAMSETE(Chassis& chassis, const Velocity2d& target, ControllerState& state, const ControllerGains& gains);
Velocity2d LTV_LQR(Chassis& chassis, const Velocity2d& target, ControllerState& state, const ControllerGains& gains);