#include "../ark_custom/include/hardware/motorgroup.h"
#include "v5_vcs.h"
#include <vector>

using namespace std;

int fabs(int x){
    if(x < 0) return -x;
    return x;
}
bool neg(int x){
    return x > 0;
}

// The intention of a custom Motor Class is to allow more control over individual motors.
// For example, it will allow built-in PID control, which can be used for more precise movements. 
Ark1DMotorGroup::Ark1DMotorGroup(std::vector<int*> motors, ArkPID* pid) {
    this->ports = motors;
    this->motors = std::vector<vex::motor*>();
    for (int* port : motors) {
        this->motors.push_back(new vex::motor(fabs(*port), neg(*port)));
    }
    this->pid = pid;
};
void Ark1DMotorGroup::stop() {
    for (vex::motor* motor : motors) {
        motor->stop();
    }
};
void Ark1DMotorGroup::reset(double position) {
    for (vex::motor* motor : motors) {
        motor->setPosition(position, vex::rotationUnits::deg);
    }
};
double Ark1DMotorGroup::rotation() {
    double sum = 0;
    for (vex::motor* motor : motors) {
        sum += motor->position(vex::rotationUnits::deg);
    }
    return sum;
};
void Ark1DMotorGroup::tick() {
    double output = pid->calculate(rotation());
    for (vex::motor* motor : motors) {
        motor->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
    }
};
void Ark1DMotorGroup::moveTo(double target) {
    pid->setTarget(target);
};

/*
* 2D Motor Groups are motors stored in 2 dimensions: [[motor, motor, motor], [motor, motor, motor]].
* This allows unique control over all the motor groups more easily. For example,
* a 3-motor differential drivetrain would be a 3x2 2D motor group. Through this,
* it creates infinite possibilities of motor configurations.
* Yes, this is an original idea from Arklib!
*/

Ark2DMotorGroup::Ark2DMotorGroup(std::vector<std::vector<int*>> motors, std::vector<ArkPID*> pids){
    this->ports = motors;
    this->motors = std::vector<std::vector<vex::motor*>>();
    for (std::vector<int*> sector : motors){
        std::vector<vex::motor*> motorgroup;
        for (int* port : sector) {
            motorgroup.push_back(new vex::motor(fabs(*port), neg(*port)));
        }
        this->motors.push_back(motorgroup);
    }
    this->pids = pids;
}
void Ark2DMotorGroup::moveTo(int sector, double target){
    pids[sector]->setTarget(target);
}
void Ark2DMotorGroup::moveTo(std::vector<double> targets){
    for(int i=0;i<targets.size();i++){
        this->pids[i]->setTarget(targets[i]);
    }
}
void Ark2DMotorGroup::tick(){
    std::vector<double> rotations = rotation();
    std::vector<char> is_disabled(this->disabled.size(), 0);

    for (int k = 0; k < (int)this->disabled.size(); ++k) {
        int d = this->disabled[k];
        if (d >= 0 && d < (int)this->disabled.size()) {
            is_disabled[d] = 1;
        }
    }

    for (int i = 0; i < (int)this->disabled.size(); ++i) {
        if (!is_disabled[i]) {
            tick(i, rotations[i]);
        }
    }
}
void Ark2DMotorGroup::tick(int sector){
    double rotation = this->rotation(sector);
    double output = pids[sector]->calculate(rotation);
    for (vex::motor* motor : motors[sector]) {
        motor->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
    }
}
void Ark2DMotorGroup::tick(int sector, double rotation){
    double output = pids[sector]->calculate(rotation);
    for (vex::motor* motor : motors[sector]) {
        motor->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
    }
}
void Ark2DMotorGroup::stop(int sector) {
    for (vex::motor* motor : motors[sector]) {
        motor->stop();
    }
};
void Ark2DMotorGroup::stop() {
    for(std::vector<vex::motor*>& motorgroup : motors){
        for (vex::motor* motor : motorgroup) {
            motor->stop();
        }
    }
};
double Ark2DMotorGroup::rotation(int sector){
    double sum = 0;
    for (vex::motor* motor : motors[sector]) {
        sum += motor->position(vex::rotationUnits::deg);
    }
    return sum;
}
std::vector<double> Ark2DMotorGroup::rotation(){
    std::vector<double> rotations;
    for(int i=0;i<motors.size();i++){
        rotations.push_back(rotation(i));
    }
    return rotations;
}
void Ark2DMotorGroup::reset(int sector, double position) {
    for (vex::motor* motor : motors[sector]) {
        motor->setPosition(position, vex::rotationUnits::deg);
    }
};
void Ark2DMotorGroup::resetAll(std::vector<double> positions){
    int i = 0;
    for(std::vector<vex::motor*> motorgroup : motors){
        for (vex::motor* motor : motorgroup) {
            motor->setPosition(positions[i], vex::rotationUnits::deg);
        };
        i++;
    }
}
void Ark2DMotorGroup::disable(int sector) {
    if (sector >= 0 && sector < (int)this->disabled.size()) {
        this->disabled[sector] = 1;
    }
}
void Ark2DMotorGroup::disable(const std::vector<int>& sectors) {
    for (int i = 0; i < (int)sectors.size(); ++i) {
        int s = sectors[i];
        if (s >= 0 && s < (int)this->disabled.size()) {
            this->disabled[s] = 1;
        }
    }
}
void Ark2DMotorGroup::enable(int sector) {
    if (sector >= 0 && sector < (int)this->disabled.size()) {
        this->disabled[sector] = 0;
    }
}
void Ark2DMotorGroup::enable(const std::vector<int>& sectors) {
    for (int i = 0; i < (int)sectors.size(); ++i) {
        int s = sectors[i];
        if (s >= 0 && s < (int)this->disabled.size()) {
            this->disabled[s] = 0;
        }
    }
}