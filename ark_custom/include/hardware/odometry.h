#define _USE_MATH_DEFINES
#include "v5.h"
#include <array>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <utility>
#include <vector>
#include <type_traits>

#include "../util/pose.h"
#include "../util/filters.h"
#include "../util/units.hpp"
#include "../motion/PID.h"
#include "../hardware/motorgroup.h"

class TrackingWheel : public PositionSource {
public:
    vex::rotation tracker;
    double wheel = 2.0;
    double angle = 0.0;
    double offsets[2] = {0.0, 0.0};
    double lastRotation = 0.0;
    double lastHeading = 0.0;

    ~TrackingWheel();
    TrackingWheel(int port, double wheel, double offsets[3]);
    std::array<double, 2> gains(double dHeading);
    void tick() override;
    void reset() override;
};

class DifferentialOdometry : public HeadingSource, public PositionSource {
public:
    using HeadingSource::thetaGains;
    Ark2DMotorGroup& Drivetrain;
    std::vector<double> lastPosition = {0.0, 0.0};
    double lastHeading = 0.0;
    double wheelBase = 1.0;
    double trackWidth = 1.0;
    double wheelDiameter = 0.0;
    double gearRatio = 1.0;

    double multiplier = 1.0;

    ~DifferentialOdometry();
    DifferentialOdometry(Ark2DMotorGroup& drivetrain, double wheel, double wheelBase, double trackWidth, double gearRatio=1.0);

    std::array<double, 2> gains(double dHeading);
    double thetaGains(double lastHeading, bool degrees=true) override;
    void tick() override;
    void reset() override;
};

class Inertial : public HeadingSource {
public:
    vex::inertial inertial;
    using HeadingSource::thetaGains;

    Inertial(int port);
    double thetaGains(double lastHeading, bool degrees=true) override;
    void calibrate() override;
};

class Odometry {
private:
    std::vector<Source<HeadingSource>> headingSources;
    std::vector<Source<PositionSource>> positionSources;
    OdometryFilter& filter;
public:
    Odometry(
        std::vector<Source<HeadingSource>> heading,
        std::vector<Source<PositionSource>> position,
        OdometryFilter& filter
    );

    void tick();
    double globalX = 0, globalY = 0, globalTheta = 0;
    
    std::array<double, 3> getPose();
    Pose2d getPose(bool pose);
    void resetDevices();
    void setX(double x);
    void setY(double y);
    void setT(double theta);
    void setXY(double x, double y);
    void setXYT(double x, double y, double theta);
    void setPose(double x, double y, double theta);
};