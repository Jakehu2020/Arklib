#pragma once

#include "../ark_custom/include/hardware/motors.hpp"
#include "v5_vcs.h"
#include <cmath>

class ArkPID;


// The intention of a custom Motor Class is to allow more control over individual motors.
// For example, it will allow built-in PID control, which can be used for more precise movements. 
class ArkMotor {
    public:
        vex::motor* motor;
    // private:

    ArkMotor(int port, double ratio, ArkPID* pid) {
        motor = new vex::motor(std::abs(port), vex::ratio6_1, port < 0);
    };
};