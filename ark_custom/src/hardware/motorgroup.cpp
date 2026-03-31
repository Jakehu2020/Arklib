#include "../ark_custom/include/hardware/motorgroup.h"
#include "v5_vcs.h"
#include <cmath>

// The intention of a custom Motor Class is to allow more control over individual motors.
// For example, it will allow built-in PID control, which can be used for more precise movements. 
ArkMotorGroup::ArkMotorGroup(std::vector<int*> motors, ArkPID* pid, int priority=0) {
    this->ports = motors;
    this->motors = std::vector<vex::motor*>();
    for (int* port : motors) {
        this->motors.push_back(new vex::motor(*port));
    }
    this->pid = pid;
    this->priority = priority;
};
void ArkMotorGroup::stop() {
    for (vex::motor* motor : motors) {
        motor->stop();
    }
};
void ArkMotorGroup::reset(double position) {
    for (vex::motor* motor : motors) {
        motor->setPosition(position, vex::rotationUnits::deg);
    }
};
double ArkMotorGroup::rotation() {
    return motors[priority]->position(vex::rotationUnits::deg);
};
void ArkMotorGroup::tick(double target) {
    double output = pid->calculate(rotation());
    for (vex::motor* motor : motors) {
        motor->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
    }
};
void ArkMotorGroup::moveTo(double target) {
    pid->setTarget(target);
};