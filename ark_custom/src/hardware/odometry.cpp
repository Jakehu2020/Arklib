#include "../ark_custom/include/hardware/odometry.h"
#include "v5.h"
#include <array>
#include <cmath>
#include <cstdlib>

/*
The idea is to have:
tracking wheels (wheel and angle), differential drivetrain odometry, x-drive odometry, etc (individual classes).
Each of these individual classes will calculate odom gains (assuming no inertial rotation), and then we can
filter through these to find the best, most accurate portrayal of the position.

Then we can connect them all in a single "Tracker" class that filters through odometry
and finds the correct odom gains, and rotates it by the inertial heading.
Filters can include KF, EKF, clamping etc.
*/

TrackingWheel::TrackingWheel(int port, double wheel, double offsets[3])
    : tracker(std::abs(port), port < 0),
        wheel(wheel),
        angle(offsets[2]),
        offsets{offsets[0], offsets[1]},
        lastRotation(0.0),
        lastHeading(0.0)
{
    tracker.setPosition(0.0, vex::degrees);
}

std::array<double, 2> TrackingWheel::gains(double dHeading)
{
    double currentRotation = tracker.position(vex::degrees);
    double deltaDeg = currentRotation - lastRotation;
    lastRotation = currentRotation;

    double displacement = deltaDeg * (M_PI * wheel / 360.0);
    double dx = displacement * std::sin(angle) - offsets[1] * dHeading;
    double dy = displacement * std::cos(angle) + offsets[0] * dHeading;

    dx -= -offsets[1] * dHeading;
    dy -= offsets[0] * dHeading;

    return {dx, dy};
}

DifferentialOdometry::DifferentialOdometry(Ark2DMotorGroup& drivetrain, double wheel, double wheelBase, double trackWidth, double gearRatio)
    : Drivetrain(drivetrain), wheelBase(wheelBase), trackWidth(trackWidth), wheelDiameter(wheel),
    gearRatio(gearRatio), multiplier(M_PI * wheelDiameter / (gearRatio * 360.0))
{
    Drivetrain.resetAll({0.0, 0.0});
}

std::array<double, 2> DifferentialOdometry::gains(double dHeading)
{
    std::vector<double> positions = Drivetrain.rotation();
    double deltaY = 0.0;
    double deltaPos[2] = {
        multiplier * (positions[0] - lastPosition[0]),
        multiplier * (positions[1] - lastPosition[1])
    };
    
    if (std::abs(dHeading) < 0.001) {
        // Straight Line
        deltaY = (deltaPos[0] + deltaPos[1]) / 2.0;
    } else {
        // Arc-based Geometry
        deltaY = std::sin(dHeading / 2.0) * ((deltaPos[0] + deltaPos[1]) / dHeading + wheelBase);
    }
    
    double angle = lastHeading + dHeading / 2.0;
    return {deltaY * std::sin(angle), deltaY * std::cos(angle)};
}

double DifferentialOdometry::thetaGains()
{
    std::vector<double> positions = Drivetrain.rotation();

    double deltaPos[2] = {
        multiplier * (positions[0] - lastPosition[0]),
        multiplier * (positions[1] - lastPosition[1])
    };

    return (deltaPos[1] - deltaPos[0])/trackWidth * 180.0/M_PI;
}

void DifferentialOdometry::tick(double heading)
{
    std::vector<double> positions = Drivetrain.rotation();
    lastHeading = heading;
    lastPosition = positions;
}

Inertial::Inertial(int port)
    : inertial(vex::inertial(std::abs(port), port > 0 ? vex::turnType::right : vex::turnType::left))
{}

double Inertial::thetaGains()
{
    return inertial.heading();
}

void Inertial::calibrate()
{
    inertial.calibrate();
}

// ODOMETRY

Odometry::Odometry(
    std::vector<Source<HeadingSource>> heading,
    std::vector<Source<PositionSource>> position
) : headingSources(heading), positionSources(position) {}

double Odometry::heading(){
    double x = 0.0, y = 0.0;

    for (auto& s : headingSources) {
        double h = s.source->thetaGains();
        x += std::cos(h) * s.weight;
        y += std::sin(h) * s.weight;
    }
    
    return std::atan2(y, x);
}

std::array<double, 2> Odometry::gains() {
    double dx = 0.0, dy = 0.0, totalWeight = 0.0;

    for (auto& s : positionSources) {
        std::array<double, 2> gain = s.source->gains(globalTheta - lastTheta); // only delta heading is used anyway; all sources only use local information, so this makes no difference.
        dx += gain[0] * s.weight;
        dy += gain[1] * s.weight;
        totalWeight += s.weight;
    }

    return {dx / totalWeight, dy / totalWeight};
}

void Odometry::tick(){
    double dTheta = heading();
    lastTheta = globalTheta;
    globalTheta += dTheta;

    std::array<double, 2> gain = gains();
    globalX += gain[0];
    globalY += gain[1];
}

std::array<double, 3> Odometry::getPose(){
    return {globalX, globalY, globalTheta};
}

void Odometry::setX(double x){
    globalX = x;
}

void Odometry::setY(double y){
    globalY = y;
}

void Odometry::setT(double theta){
    globalTheta = theta;
}

void Odometry::setXY(double x, double y){
    globalX = x;
    globalY = y;
}

void Odometry::setXYT(double x, double y, double theta){
    globalX = x;
    globalY = y;
    globalTheta = theta;
}

void Odometry::setPose(double x, double y, double theta){
    globalX = x;
    globalY = y;
    globalTheta = theta;
}
