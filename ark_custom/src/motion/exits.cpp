#include "../ark_custom/include/motion/exits.h"
#include "v5_vcs.h"

errorTimeout::errorTimeout(double radius, double timer_sec, double timeout_sec)
    : difference(radius), time(timer_sec * 1000.0), maxtimeout(timeout_sec * 1000.0)
{
    target.setX(0);
    target.setY(0);
};

void errorTimeout::reset()
{
    target.setX(0);
    target.setY(0);
    started = false;
    timestart = 0;
    smalltimestart = 0;
};

bool errorTimeout::operator()(Pose2d position)
{
    double currentTime = vex::timer::system();

    if (!started)
    {
        started = true;
        timestart = currentTime;
        smalltimestart = 0;
        return false;
    };

    if (currentTime - timestart >= maxtimeout)
    {
        return true;
    }

    double currentDistance = position.getDistance(target);

    if (currentDistance < difference)
    {
        if (smalltimestart == 0)
        {
            smalltimestart = currentTime;
        }
        else if (currentTime - smalltimestart >= time)
        {
            timestart = 0;
            smalltimestart = 0;
            return true;
        }
    }
    else
    {
        smalltimestart = 0;
    }

    return false;
};

void errorTimeout::setTarget(double x, double y)
{
    target.setX(x);
    target.setY(y);
};