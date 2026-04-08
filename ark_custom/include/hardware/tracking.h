#include "../units.hpp"

class Odometry {
    public:
        vex::motor* motor;
        ArkPID* pid;
        
        Odometry(int port, double wheelDiameter);
        int rotation();
        void moveTo(double target);
        void tick(double target);
        void stop();
        void reset(double position);
};