# TODO

Here is a list of possible additions to the project, sorted by priority.

### Hardware:
1. General Motor Control (adaption with PID class)
2. Motor Groups (Motor class extension that holds multiple motors)
3. General Tracking Wheel Hardware (e.g. ADI-encoders, Rotation Sensors)
4. Differential Drivetrain class -> "Chassis" (chassis should contain custom software motions)

### Software:
1. General PID Control
2. Drivetrain Odometry from Motor Encoders
3. Odometry Pod-received Odometry (with offsets!)
4. Drivetrain Simple PID from Odometry (Drive Forward + TurntoHeading)
5. Lemlib MoveToPoint
6. Brain and Controller Auton Selector
7. On-the-page PID tuning (to reduce the need to continuously upload for PID)