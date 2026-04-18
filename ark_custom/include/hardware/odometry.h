#include "../units.hpp"
#include "../motion/PID.h"

#define _USE_MATH_DEFINES
#include "v5.h"
#include <array>
#include <cmath>
#include <cstdlib>

#include "../units.hpp"
#include "../motion/PID.h"
#include "../hardware/motorgroup.h"

template<typename T>
struct Source {
    T* source;
    double weight;
};

class HeadingSource {
public:
    virtual double thetaGains();
    virtual void calibrate();
    virtual ~HeadingSource() = default;
};

class PositionSource {
public:
    virtual std::array<double, 2> gains(double heading);
    virtual ~PositionSource() = default;
    virtual void tick(double heading) {};
};

class TrackingWheel : public PositionSource {
public:
    vex::rotation tracker;
    double wheel = 2.0;
    double angle = 0.0;
    double offsets[2] = {0.0, 0.0};
    double lastRotation = 0.0;
    double lastHeading = 0.0;

    TrackingWheel(int port, double wheel, double offsets[3]);
    std::array<double, 2> gains(double dHeading) override;
    void tick(double heading) override;
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

    DifferentialOdometry(Ark2DMotorGroup& drivetrain, double wheel, double wheelBase, double trackWidth, double gearRatio=1.0);

    std::array<double, 2> gains(double dHeading) override;
    double thetaGains() override;
    void tick(double heading) override;
};

class Inertial : public HeadingSource {
public:
    vex::inertial inertial;
    using HeadingSource::thetaGains;

    Inertial(int port);
    double thetaGains() override;
    void calibrate() override;
};

class Odometry {
private:
    std::vector<Source<HeadingSource>> headingSources;
    std::vector<Source<PositionSource>> positionSources;

public:
    Odometry(
        std::vector<Source<HeadingSource>> heading,
        std::vector<Source<PositionSource>> position
    );
    double globalX = 0.0, globalY = 0.0, globalTheta = 0.0;
    double lastTheta = 0.0;

    double heading();
    std::array<double, 2> gains();
    void tick();
    std::array<double, 3> getPose();
    void setX(double x);
    void setY(double y);
    void setT(double theta);
    void setXY(double x, double y);
    void setXYT(double x, double y, double theta);
    void setPose(double x, double y, double theta);
};