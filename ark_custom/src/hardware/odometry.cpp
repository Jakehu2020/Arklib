#include "v5_vcs.h"
// #include "../motion/PID.h"
// #include "../ark_custom/include/hardware/motorgroup.h"

// The idea is to have:
// tracking wheels (wheel and angle)
// differential drivetrain odometry
// x-drive odometry
// etc (individual classes)
// Each of these individual classes will calculate odom gains, and then we can
// filter through these to find the best, most accurate portrayal of the position.

// Then we can connect them all in a single "Tracker" class that filters through odometry
// and finds the correct position in getPose() -> {x, y, theta}.

// class Tracker {
//     public:
//         Tracker(int port, double wheel, int angle){

//         }
// }
// class DifferentialOdometry {
//     public:
//         Ark2DMotorGroup* Drivetrain;

//         DifferentialOdometry(Ark2DMotorGroup& drivetrain, double wheel, double gearRatio){
//             Drivetrain = drivetrain;
//         }
        
// }