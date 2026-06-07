#include "../ark_custom/include/util/pose.h"
#include <cmath>

Pose2d::Pose2d() : m_x(0.0), m_y(0.0), m_theta(0.0) {}

Pose2d::Pose2d(double x, double y, double theta)
    : m_x(x), m_y(y), m_theta(normalizeAngle(theta)) {}

double Pose2d::normalizeAngle(double angle) const {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0) a += 2.0 * M_PI;
    return a - M_PI;
}

double Pose2d::getDistance(const Pose2d& other) const {
    double dx = other.m_x - m_x;
    double dy = other.m_y - m_y;
    return std::sqrt(dx * dx + dy * dy);
}

Pose2d Pose2d::transformBy(const Pose2d& other) const {
    double cosTheta = std::cos(m_theta);
    double sinTheta = std::sin(m_theta);
    return Pose2d(
        m_x + (other.m_x * cosTheta - other.m_y * sinTheta),
        m_y + (other.m_x * sinTheta + other.m_y * cosTheta),
        m_theta + other.m_theta
    );
}

Pose2d Pose2d::relativeTo(const Pose2d& other) const {
    double transformX = m_x - other.m_x;
    double transformY = m_y - other.m_y;
    double cosTheta = std::cos(other.m_theta);
    double sinTheta = std::sin(other.m_theta);
    return Pose2d(
        transformX * cosTheta + transformY * sinTheta,
        -transformX * sinTheta + transformY * cosTheta,
        m_theta - other.m_theta
    );
}

Pose2d Pose2d::operator+(const Pose2d& other) const { return transformBy(other); }
bool Pose2d::operator==(const Pose2d& other) const {
    return (m_x == other.m_x) && (m_y == other.m_y) && (m_theta == other.m_theta);
}
bool Pose2d::operator!=(const Pose2d& other) const { return !(*this == other); }

Velocity2d::Velocity2d()
    : Pose2d(), v(0.0), omega(0.0), time(0.0) {}

Velocity2d::Velocity2d(const Pose2d& pose, double linearVel, double angularVel, double time)
    : Pose2d(pose), v(linearVel), omega(angularVel), time(time) {}

Velocity2d::Velocity2d(double x, double y, double theta, double linearVel, double angularVel, double time)
    : Pose2d(x, y, theta), v(linearVel), omega(angularVel), time(time) {}

Velocity2d Velocity2d::predictState(double dt) const {
    double nextTheta = getTheta() + omega * dt;
    double nextX = getX();
    double nextY = getY();

    if (std::abs(omega) < 1e-4) {
        nextX += v * std::cos(getTheta()) * dt;
        nextY += v * std::sin(getTheta()) * dt;
    } else {
        double radius = v / omega;
        nextX += radius * (std::sin(nextTheta) - std::sin(getTheta()));
        nextY += radius * (std::cos(getTheta()) - std::cos(nextTheta));
    }

    return Velocity2d(nextX, nextY, nextTheta, v, omega, time + dt);
}