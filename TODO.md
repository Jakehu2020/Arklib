# TODO

Here is a list of possible additions to the project, sorted by priority.

### Hardware:
1. Chassis: 2D motor group + odometry should contain custom software motions
2. Odometry - More tools, such as GPS, X-Drive Odometry, Holonomic Odom (using drift wheel motor encoder), etc
3. Motor Storage Structure for multi-usage of motors without re-definition.

### Software:
1. Basic Drive-controllers with changeable curvature-scaling and drift: Arcade, Tank, Cheesy
2. Linear/Bezier Motion Profiles
3. General Path-following Motion Controller on Chassis
4. Kalman-Filtered Odometry
5. Simple Lyapunov-Stability-Equation-based Motion Controllers
6. Brain and Controller Auton Selector
7. Auto-PID tuning (to reduce the need to continuously upload for PID)

### External:
1. Convert `Eigen` features into custom library features for abstraction, speed, and library size (E.g. LU Solver, Cholesky decomposition, Discrete Algebraic Riccati Equation/DARE)
2. Auto-downloader script to automatically download all codes to different ports on the brain
3. Script to read output from the brain to show the perceived robot path on odometry, which can be shown on a website for debugging.
4. Document/Comment all code to explain their functionality as per \<G4\>