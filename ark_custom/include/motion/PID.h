#pragma once

#include <string>
#include "v5.h"
#include "v5_vcs.h"

class ArkPID {
    private:
        double target;

        double lastError;
        double lastState;
        double integral;
        double lastOutput;

        uint64_t lastTime;
        bool firstRun;
        
    public:
        double kP, kI, kD, kF;

        double outputMax;
        double outputMin;
        double integralLimit;
        double integralZone;
        double tolerance;
        double slewRate;
    
        ArkPID(double p, double i, double d, double f);
        void setConstants(double p, double i, double d, double f);
        void setTarget(double t);
        void reset();
        bool atTarget(double state);
        double calculate(double state);
};