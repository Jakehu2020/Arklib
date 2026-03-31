#include "../ark_custom/include/hardware/motors.h"
#include "v5_vcs.h"
#include <cmath>

// The intention of a custom Motor Class is to allow more control over individual motors.
// For example, it will allow built-in PID control, which can be used for more precise movements. 
ArkMotor::ArkMotor(int port, double ratio, ArkPID* pid) {
    motor = new vex::motor(std::abs(port), vex::ratio6_1, port < 0);
    this->pid = pid;
};
int ArkMotor::rotation() {
    return motor->position(vex::degrees);
};
void ArkMotor::moveTo(double target) {
    pid->setTarget(target);
};
void ArkMotor::tick(double target){
    double output = pid->calculate(rotation());
    motor->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
};
void ArkMotor::stop() {
    motor->stop();
};
void ArkMotor::reset(double position) {
    motor->setPosition(position, vex::degrees);
};
// NOTE: Add more functionality in the future.
// For example, Asymptotic Gains, Rotation Sensors, etc.