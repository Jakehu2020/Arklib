#include "../ark_custom/include/hardware/odometry.h"
#include "v5.h"
#include <array>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <utility>
#include <type_traits>

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
        lastRotation(0.0)
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
    
    return {dx, dy};
}
void TrackingWheel::reset(){
    tracker.setPosition(0.0, vex::degrees);
    lastRotation = 0.0;
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
        double r = (deltaPos[0] + deltaPos[1]) / (2.0 * dHeading);
        deltaY = 2.0 * r * std::sin(dHeading / 2.0);
    }
    
    double angle = dHeading / 2.0;
    return {deltaY * std::sin(angle), deltaY * std::cos(angle)};
}

double DifferentialOdometry::thetaGains(double lastHeading, bool degrees)
{
    std::vector<double> positions = Drivetrain.rotation();

    double deltaPos[2] = {
        multiplier * (positions[0] - lastPosition[0]),
        multiplier * (positions[1] - lastPosition[1])
    };

    return (deltaPos[1] - deltaPos[0])/trackWidth * (degrees ? 1.0 : 180.0/M_PI );
}

void DifferentialOdometry::reset()
{
    Drivetrain.resetAll({0.0, 0.0});
    lastPosition = {0.0, 0.0};
}

void DifferentialOdometry::tick()
{
    std::vector<double> positions = Drivetrain.rotation();
    lastPosition = positions;
}

Inertial::Inertial(int port)
    : inertial(vex::inertial(std::abs(port), port > 0 ? vex::turnType::right : vex::turnType::left))
{}

double Inertial::thetaGains(double lastHeading, bool degrees)
{
    return degrees ? inertial.heading() : inertial.heading() * M_PI / 180.0 - lastHeading;
}

void Inertial::calibrate()
{
    inertial.calibrate();
}

// ODOMETRY
Odometry::Odometry(
    std::vector<Source<HeadingSource>> heading,
    std::vector<Source<PositionSource>> position,
    OdometryFilter& filter
) : headingSources(heading), positionSources(position), filter(filter) {}

void Odometry::tick() {
    double newTheta = filter.filterHeading(headingSources);
    double dTheta = newTheta - globalTheta;

    globalTheta = newTheta;

    auto gain = filter.filterPosition(positionSources, dTheta);
    globalX += gain[0];
    globalY += gain[1];
}

std::array<double, 3> Odometry::getPose(){
    return {globalX, globalY, globalTheta};
}

Pose2d Odometry::getPose(bool pose){
    Pose2d position;
    position.setAll(globalX, globalY, globalTheta);
    return position;
}

void Odometry::resetDevices(){
    for (auto& s : positionSources) {
        s.source->reset();
    }
    for (auto& s : headingSources) {
        s.source->reset();
    }
}

void Odometry::setX(double x){
    this->resetDevices();
    globalX = x;
}

void Odometry::setY(double y){
    this->resetDevices();
    globalY = y;
}

void Odometry::setT(double theta){
    this->resetDevices();
    globalTheta = theta;
}

void Odometry::setXY(double x, double y){
    this->resetDevices();
    globalX = x;
    globalY = y;
}

void Odometry::setXYT(double x, double y, double theta){
    this->resetDevices();
    globalX = x;
    globalY = y;
    globalTheta = theta;
}

void Odometry::setPose(double x, double y, double theta){
    this->resetDevices();
    globalX = x;
    globalY = y;
    globalTheta = theta;
}
