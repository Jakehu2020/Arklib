#include "../motion/PID.h"
#include "./motors.h"
#include <vector>

class ArkMotorGroup {
    public:
        int priority = 0;
        std::vector<int*> ports;
        std::vector<vex::motor*> motors;
        ArkPID* pid;

        ArkMotorGroup(std::vector<int*> motors, ArkPID* pid, int priority);
        void moveTo(double target);
        void tick(double target);
        void stop();
        double rotation();
        void reset(double position);
};