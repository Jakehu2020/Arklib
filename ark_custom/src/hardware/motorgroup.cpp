#include "../ark_custom/include/hardware/motorgroup.h"
#include <cstdlib>
#include <vector>

int fabs(int x){
    if(x < 0) return -x;
    return x;
}

Ark1DMotorGroup::Ark1DMotorGroup(std::vector<int> ports) {
    for (int port : ports) {
        motors.push_back(new vex::motor(fabs(port), port < 0));
    }
}

void Ark1DMotorGroup::move(double velocity) {
    for (vex::motor* motor : motors) {
        motor->spin(vex::directionType::fwd, velocity, vex::velocityUnits::rpm);
    }
}

void Ark1DMotorGroup::move_voltage(double voltage) {
    for (vex::motor* motor : motors) {
        motor->spin(vex::directionType::fwd, voltage, vex::voltageUnits::volt);
    }
}

void Ark1DMotorGroup::stop() {
    for (vex::motor* motor : motors) {
        motor->stop();
    }
}

double Ark1DMotorGroup::rotation() {
    double sum = 0;
    for (vex::motor* motor : motors) {
        sum += motor->position(vex::rotationUnits::deg);
    }
    return sum / motors.size();
}

void Ark1DMotorGroup::reset(double position) {
    for (vex::motor* motor : motors) {
        motor->setPosition(position, vex::rotationUnits::deg);
    }
}

Ark2DMotorGroup::Ark2DMotorGroup(std::vector<std::vector<int>> ports) {
    for (auto& sector : ports) {
        std::vector<vex::motor*> motorGroup;
        for (int port : sector) {
            motorGroup.push_back(new vex::motor(fabs(port), port < 0));
        }
        motors.push_back(motorGroup);
    }
}

void Ark2DMotorGroup::move(int sector, double velocity) {
    for (vex::motor* motor : motors[sector]) {
        motor->spin(vex::directionType::fwd, velocity, vex::velocityUnits::rpm);
    }
}

void Ark2DMotorGroup::move(std::vector<double> velocities) {
    for (int i = 0; i < (int)motors.size() && i < (int)velocities.size(); ++i) {
        move(i, velocities[i]);
    }
}

void Ark2DMotorGroup::move_voltage(int sector, double voltage) {
    for (vex::motor* motor : motors[sector]) {
        motor->spin(vex::directionType::fwd, voltage, vex::voltageUnits::volt);
    }
}

void Ark2DMotorGroup::move_voltage(std::vector<double> voltages) {
    for (int i = 0; i < (int)motors.size() && i < (int)voltages.size(); ++i) {
        move_voltage(i, voltages[i]);
    }
}

void Ark2DMotorGroup::stop(int sector) {
    for (vex::motor* motor : motors[sector]) {
        motor->stop();
    }
}

void Ark2DMotorGroup::stop() {
    for (auto& motorGroup : motors) {
        for (vex::motor* motor : motorGroup) {
            motor->stop();
        }
    }
}

double Ark2DMotorGroup::rotation(int sector) {
    double sum = 0;
    for (vex::motor* motor : motors[sector]) {
        sum += motor->position(vex::rotationUnits::deg);
    }
    return sum / motors[sector].size();
}

std::vector<double> Ark2DMotorGroup::rotation() {
    std::vector<double> rotations;
    for (int i = 0; i < (int)motors.size(); ++i) {
        rotations.push_back(rotation(i));
    }
    return rotations;
}

void Ark2DMotorGroup::reset(int sector, double position) {
    for (vex::motor* motor : motors[sector]) {
        motor->setPosition(position, vex::rotationUnits::deg);
    }
}

void Ark2DMotorGroup::resetAll(std::vector<double> positions) {
    for (int i = 0; i < (int)motors.size() && i < (int)positions.size(); ++i) {
        reset(i, positions[i]);
    }
}
