#include "../motion/PID.h"
#include "./motors.h"
#include <vector>

class Ark1DMotorGroup {
    public:
        std::vector<int*> ports;
        std::vector<vex::motor*> motors;
        ArkPID* pid;

        Ark1DMotorGroup(std::vector<int*> motors, ArkPID* pid);
        void moveTo(double target);
        void tick();
        void stop();
        double rotation();
        void reset(double position);
};

class Ark2DMotorGroup {
    public:
        std::vector<std::vector<int*>> ports;
        std::vector<std::vector<vex::motor*>> motors;
        std::vector<ArkPID*> pids;
        std::vector<char> disabled;

        Ark2DMotorGroup(std::vector<std::vector<int*>> motors, std::vector<ArkPID*> pids);
        void moveTo(int sector, double target);
        void moveTo(std::vector<double> targets);
        void tick();
        void tick(int sector);
        void tick(int sector, double rotation);
        void stop(int sector);
        void stop();
        double rotation(int sector);
        std::vector<double> rotation();
        void reset(int sector, double position);
        void resetAll(std::vector<double> position);
        void disable(int sector);
        void disable(const std::vector<int>& sectors);
        void enable(int sector);
        void enable(const std::vector<int>& sectors);
};