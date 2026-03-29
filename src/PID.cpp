#pragma once

#include "vex.h"
#include "v5.h"

// IMPORTANT: THIS IS UNFINISHED AND REQUIRES DOCUMENTATION.
// Documentation will be added once the class is finalized. Many algorithms are still being tested, so the interface may change in the future.

// Simple helpers (std doesn't work on VEX)
inline double absd(double x) { return x < 0 ? -x : x; }

template<typename T>
T clamp(const T& value, const T& low, const T& high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

class PID {
    public:
        double kP, kI, kD, kF;

        double outputMax;
        double outputMin;
        double integralLimit;
        double integralZone;
        double tolerance;
        double slewRate;

    private:
        double target;

        double lastError;
        double lastState;
        double integral;
        double lastOutput;

        uint64_t lastTime;
        bool firstRun;
    
    PID(double p, double i, double d, double f);
    void setConstants(double p, double i, double d, double f);
    void setTarget(double t);
    void reset();
    bool atTarget(double state);
    double calculate(double state);
};

/**
 * @brief Constructs a PID controller object.
 *
 * @param p Proportional gain
 * @param i Integral gain
 * @param d Derivative gain
 * @param f Feedforward gain (default is 0)
 *
 * Initializes the PID controller with the specified values.
 */

PID::PID(double p, double i, double d, double f = 0)
    : kP(p), kI(i), kD(d), kF(f),
          outputMax(12.0), outputMin(-12.0),
          integralLimit(1000), integralZone(1e9),
          tolerance(1.0), slewRate(1e9),
          target(0), lastError(0), lastState(0),
          integral(0), lastOutput(0),
          lastTime(vex::timer::systemHighResolution()),
          firstRun(true) {}

void PID::setConstants(double p, double i, double d, double f = 0) {
    kP = p; kI = i; kD = d; kF = f;
}

void PID::setTarget(double t) {
    target = t;
}

void PID::reset() {
    lastError = 0;
    lastState = 0;
    integral = 0;
    lastOutput = 0;
    lastTime = vex::timer::systemHighResolution();
    firstRun = true;
}

bool PID::atTarget(double state) {
    return absd(target - state) <= tolerance;
}

double PID::calculate(double state) {
    uint64_t now = vex::timer::systemHighResolution();
    double dt = (now - lastTime) / 1000.0;
    if (dt <= 0) dt = 0.001;

    double error = target - state;
    double P = kP * error;

    if (absd(error) < integralZone) {
        integral += error * dt;
        integral = clamp(integral, -integralLimit, integralLimit);
    } else {
        integral = 0;
    }
    double I = kI * integral;

    double D = 0;
    if (!firstRun) {
        double velocity = (state - lastState) / dt;
        D = -kD * velocity;
    }

    double F = kF * target;
    double output = P + I + D + F;

    output = clamp(output, outputMin, outputMax);

    double delta = output - lastOutput;
    double maxDelta = slewRate * dt;
    delta = clamp(delta, -maxDelta, maxDelta);
    output = lastOutput + delta;

    lastState = state;
    lastError = error;
    lastOutput = output;
    lastTime = now;
    firstRun = false;

    return output;
}