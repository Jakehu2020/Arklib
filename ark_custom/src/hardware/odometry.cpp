#include "v5.h"
#include <array>
#include <cmath>

// #include "../motion/PID.h"
#include "../ark_custom/include/hardware/motorgroup.h"
/*
The idea is to have:
tracking wheels (wheel and angle), differential drivetrain odometry, x-drive odometry, etc (individual classes).
Each of these individual classes will calculate odom gains (assuming no inertial rotation), and then we can
filter through these to find the best, most accurate portrayal of the position.

Then we can connect them all in a single "Tracker" class that filters through odometry
and finds the correct odom gains, and rotates it by the inertial heading.
Filters can include KF, EKF, clamping etc.
*/

class Tracker {
    public:
        vex::rotation tracker;
        double wheel      = 2.0;
        double angle      = 0.0;
        double offsets[2] = {0.0, 0.0};
        double lastRotation = 0.0;
        double lastHeading  = 0.0;

        Tracker(int port, double wheel, double offsets[3])
            : tracker(std::abs(port), port < 0),
              wheel(wheel),
              angle(offsets[2]),
              offsets{offsets[0], offsets[1]},
              lastRotation(0.0),
              lastHeading(0.0)
        {}

        std::array<double, 2> gains(double heading) {
            double currentRotation = tracker.position(vex::degrees);
            double deltaDeg = currentRotation - lastRotation;
            lastRotation = currentRotation;

            double displacement = deltaDeg * (M_PI * wheel / 360.0);
            double dx = displacement * std::sin(angle);
            double dy = displacement * std::cos(angle);

            double dHeading = heading - lastHeading;
            lastHeading = heading;

            dx -= -offsets[1] * dHeading;
            dy -=  offsets[0] * dHeading;

            double worldDx = dx * std::cos(heading) - dy * std::sin(heading);
            double worldDy = dx * std::sin(heading) + dy * std::cos(heading);

            return {worldDx, worldDy};
        };
};
class DifferentialOdometry {
    public:
        Ark2DMotorGroup& Drivetrain;
        DifferentialOdometry(Ark2DMotorGroup& drivetrain, double wheel, double gearRatio)
          : Drivetrain(drivetrain) {
            Drivetrain.resetAll({0.0, 0.0});
        };

        std::array<double, 2> gains(double heading) {
            // finish this
            return {0.0, 0.0};
        };
};