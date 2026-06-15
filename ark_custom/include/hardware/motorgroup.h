#pragma once
#include "v5_vcs.h"
#include <vector>

class Ark1DMotorGroup {
    public:
        std::vector<vex::motor*> motors;

        Ark1DMotorGroup(std::vector<int> ports);
        void move(double velocity);
        void move_voltage(double voltage);
        void stop();
        double rotation();
        void reset(double position);
};

class Ark2DMotorGroup {
    public:
        std::vector<std::vector<vex::motor*>> motors;

        Ark2DMotorGroup(std::vector<std::vector<int>> ports);
        void move(int sector, double velocity);
        void move(std::vector<double> velocities);
        void move_voltage(int sector, double voltage);
        void move_voltage(std::vector<double> voltages);
        void stop(int sector);
        void stop();
        double rotation(int sector);
        std::vector<double> rotation();
        void reset(int sector, double position);
        void resetAll(std::vector<double> positions);
};
