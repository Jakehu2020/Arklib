#include "../util/pose.h"

#pragma once
#include <memory>

class Chassis;

class IExitCondition {
public:
    virtual ~IExitCondition() = default;
    virtual void reset() = 0;
    virtual bool operator()(Pose2d position) = 0;
    virtual void setTarget(double x, double y) = 0;
};

class errorTimeout : public IExitCondition{
    private:
        double difference = 0.5;
        double time = 1000.0;
        double maxtimeout = 10000.0;
        bool started = false;

        double timestart = 0;
        double smalltimestart = 0;
    public:
        Pose2d target;
        errorTimeout(double radius = 0.5, double timer_sec = 1.0, double timeout_sec = 10.0);
        void reset();
        bool operator()(Pose2d position);
        void setTarget(double x, double y);
};