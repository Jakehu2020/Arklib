#include "../ark_custom/include/motion/driver.h"
#include <cmath>

namespace controller {

DriveParams driveParams;

double redCurve(double value, double strength){
    return value * (std::exp(-strength/10) + std::exp((std::abs(value)-100)/10) * (1 - exp(-strength/10)));
}
double blueCurve(double value, double strength){
    return value * std::exp((std::abs(value) - 100) * strength / 1000);
}
double deadband(double value, double deadband){
	if (std::abs(value) < deadband) return 0.0;
	return value;
}

void arcade(Chassis& chassis, double v, double w){
    chassis.move(deadband(v, driveParams.deadband), deadband(w, driveParams.deadband));
}
void tank(Chassis& chassis, double left, double right){
    chassis.tank(deadband(left, driveParams.deadband), deadband(right, driveParams.deadband));
}

double quickStopAcc = 0.0;
double negInertiaAcc = 0.0;
double prevTurn = 0.0;
double prevThrottle = 0.0;

static double _turnRemapping(double omega) {
	double denominator = sin(3.141592654 / 2 * driveParams.turnNonlinearity);
	double firstRemapIteration =
	    sin(3.141592654 / 2 * driveParams.turnNonlinearity * omega) / denominator;
	return sin(3.141592654 / 2 * driveParams.turnNonlinearity * firstRemapIteration) / denominator;
}

// On each iteration of the drive controller (where we aren't point turning) we
// constrain the accumulators to the range [-1, 1].
static void _updateAccumulators() {
	if (negInertiaAcc > 1) {
		negInertiaAcc -= 1;
	} else if (negInertiaAcc < -1) {
		negInertiaAcc += 1;
	} else {
		negInertiaAcc = 0;
	}

	if (quickStopAcc > 1) {
		quickStopAcc -= 1;
	} else if (quickStopAcc < -1) {
		quickStopAcc += 1;
	} else {
		quickStopAcc = 0.0;
	}
}

void cheesy(Chassis& chassis, double v, double w) {
	bool turnInPlace = false;
	double linearCmd = v;
	if (fabs(v) < driveParams.deadband && fabs(w) > driveParams.deadband) {
		linearCmd = 0.0;
		turnInPlace = true;
	} else if (v - prevThrottle > driveParams.slew) {
		linearCmd = prevThrottle + driveParams.slew;
	} else if (v - prevThrottle < -(driveParams.slew * 2)) {
		linearCmd = prevThrottle - (driveParams.slew * 2);
	}

	double remappedTurn = _turnRemapping(w);

	if (turnInPlace) {
        chassis.move(0, remappedTurn * std::abs(remappedTurn));
	} else {
		double negInertiaPower = (w - prevTurn) * driveParams.negInertiaScalar;
		negInertiaAcc += negInertiaPower;
		double angularCmd = std::abs(linearCmd) * (remappedTurn + negInertiaAcc) * driveParams.sensitivity - quickStopAcc;

        chassis.move(linearCmd, angularCmd);

		_updateAccumulators();
	}

	prevTurn = w;
	prevThrottle = v;
}

} // namespace controller

/* 
Add

- Curvature Drive Controller
- Single-stick Polar Drive
- Constant-curvature Radius Controller
- Point-and-shoot / Field-oriented Differential

*/