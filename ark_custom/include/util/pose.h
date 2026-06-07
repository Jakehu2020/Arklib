#pragma once
#include <cmath>
#include <iostream>

class Pose2d {
private:
    double m_x = 0;
    double m_y = 0;
    double m_theta = 0;

    double normalizeAngle(double angle) const;
public:
    Pose2d();
    Pose2d(double x, double y, double theta);

    double getX() const { return m_x; }
    double getY() const { return m_y; }
    double getTheta() const { return m_theta; }

    void setX(double x) { m_x = x; }
    void setY(double y) { m_y = y; }
    void setTheta(double theta) { m_theta = normalizeAngle(theta); }
    void setAll(double x, double y, double theta) {
        m_x = x;
        m_y = y;
        m_theta = normalizeAngle(theta);
    }

    double getDistance(const Pose2d& other) const;
    Pose2d transformBy(const Pose2d& other) const;
    Pose2d relativeTo(const Pose2d& other) const;

    Pose2d operator+(const Pose2d& other) const;
    bool operator==(const Pose2d& other) const;
    bool operator!=(const Pose2d& other) const;

    friend std::ostream& operator<<(std::ostream& os, const Pose2d& pose);
};

class Velocity2d : public Pose2d {
private:
    double v = 0;
    double omega = 0;
    double time = 0;

public:
    Velocity2d();
    Velocity2d(const Pose2d& pose, double linearVel, double angularVel, double time = 0);
    Velocity2d(double x, double y, double theta, double linearVel, double angularVel, double time);

    double getLinearVelocity() const { return v; }
    double getAngularVelocity() const { return omega; }

    void setLinearVelocity(double linearVel) { v = linearVel; }
    void setAngularVelocity(double angularVel) { omega = angularVel; }

    Velocity2d predictState(double deltaTime) const;

    friend std::ostream& operator<<(std::ostream& os, const Velocity2d& mp);
};