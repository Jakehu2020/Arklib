#include "../motion/PID.h"

class ArkMotor {
    public:
        vex::motor* motor;
        ArkPID* pid;
        
        ArkMotor(int port, double ratio, ArkPID* pid);
        int rotation();
        void moveTo(double target);
        void tick(double target);
        void stop();
        void reset(double position);
};