#include "../ark_custom/include/motion/controllers.h"
#include <Eigen/Dense>
#include <cmath>
#include <utility>
#include <unordered_map>

static double normalizeAngle(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}

// 1. Discretize continuous state-space matrices using Taylor series expansion
static std::pair<Eigen::MatrixXf, Eigen::MatrixXf> discretizeAB(
    const Eigen::MatrixXf& contA, const Eigen::MatrixXf& contB, double dtSeconds) {
    
    int states = contA.rows();
    int inputs = contB.cols();
    
    Eigen::MatrixXf M(states + inputs, states + inputs);
    M.setZero();
    M.topLeftCorner(states, states) = contA;
    M.topRightCorner(states, inputs) = contB;
    
    Eigen::MatrixXf Mdt = M * static_cast<float>(dtSeconds);
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(M.rows(), M.cols());
    Eigen::MatrixXf M2 = Mdt * Mdt;
    
    Eigen::MatrixXf phi = I + Mdt + (M2 * 0.5f); 
    
    Eigen::MatrixXf discA = phi.topLeftCorner(states, states);
    Eigen::MatrixXf discB = phi.topRightCorner(states, inputs);
    
    return {discA, discB};
}

// 2. Fast cyclic exchange DARE solver for discrete algebraic Riccati equations
static Eigen::MatrixXf solveDARE(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const Eigen::MatrixXf &Q, const Eigen::MatrixXf &R) {
    int states = A.rows();
    
    Eigen::MatrixXf A_k = A;
    Eigen::MatrixXf G_k = B * R.llt().solve(B.transpose()); 
    Eigen::MatrixXf H_k;
    Eigen::MatrixXf H_k1 = Q;
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(states, states);

    for (int i = 0; i < 80; ++i) {
        H_k = H_k1;
        Eigen::MatrixXf W = I + G_k * H_k;
        auto W_solver = W.partialPivLu();
        Eigen::MatrixXf V_1 = W_solver.solve(A_k);
        Eigen::MatrixXf V_2 = W_solver.solve(G_k);

        G_k += A_k * V_2 * A_k.transpose();
        H_k1 = H_k + V_1.transpose() * H_k * A_k;
        A_k *= V_1;
        
        if ((H_k1 - H_k).norm() <= 1e-6f * H_k1.norm()) {
            break;
        }
    }
    return H_k1;
}

Velocity2d LTV_LQR(Chassis &chassis, const Velocity2d &target, ControllerState &state, const ControllerGains &gains, const MotionPath &path) {
    // 1. Calculate dynamic delta time (dt) across tracking loop execution steps
    double current_time = vex::timer::system() / 1000.0; // convert to seconds
    double dt = 0.02; // Default fallback to chassis task sleep loop speed
    
    if (state.count("last_time") > 0) {
        dt = current_time - state["last_time"];
    }
    state["last_time"] = current_time;
    if (dt <= 0.002 || dt > 0.1) dt = 0.02;

    // 2. Fetch Reference Trajectory Targets
    double v_ref = target.getLinearVelocity();
    double w_ref = target.getAngularVelocity();

    // 3. Transform global system tracking error into the Robot-Centric local coordinate frame
    Pose2d current = chassis.odometry.getPose(true);
    double dx = target.getX() - current.getX();
    double dy = target.getY() - current.getY();
    double error_theta = normalizeAngle(target.getTheta() - current.getTheta());

    float e_x = static_cast<float>(dx * std::cos(current.getTheta()) + dy * std::sin(current.getTheta()));
    float e_y = static_cast<float>(-dx * std::sin(current.getTheta()) + dy * std::cos(current.getTheta()));
    float e_theta = static_cast<float>(error_theta);

    Eigen::Vector3f error_vector;
    error_vector << e_x, e_y, e_theta;

    // 4. Construct Time-Varying Continuous System Matrices (A & B) based on target velocities
    float a_v_ref = (std::abs(v_ref) < 0.15) ? 0.15f : std::abs(v_ref); // prevent system singularity when stopped
    constexpr float eps = -1e-3f; // minor stabilizing diagonal damping factor
    
    Eigen::Matrix3f A;
    A << eps,  static_cast<float>(w_ref), 0.0f,
        -static_cast<float>(w_ref), eps,  a_v_ref, 
         0.0f, 0.0f, eps;
         
    Eigen::Matrix<float, 3, 2> B;
    B << 1.0f, 0.0f,
         0.0f, 0.0f,
         0.0f, 1.0f;

    // 5. Fetch LQR Tracking State Weights
    Eigen::Matrix3f Q_mat; 
    Q_mat << gains.at("q_x"), 0.0f, 0.0f,
             0.0f, gains.at("q_y"), 0.0f,
             0.0f, 0.0f, gains.at("q_theta");
    
    Eigen::Matrix2f R_mat;
    R_mat << gains.at("r_vel"), 0.0f,
             0.0f, gains.at("r_ang");

    // 6. Linearize and compute the optimal time-varying feedback gain matrix K
    auto discreteSystem = discretizeAB(A, B, dt);
    Eigen::MatrixXf X = solveDARE(discreteSystem.first, discreteSystem.second, Q_mat, R_mat);
    
    Eigen::MatrixXf K = (R_mat + discreteSystem.second.transpose() * X * discreteSystem.second).inverse() * discreteSystem.second.transpose() * X * discreteSystem.first;

    Eigen::Vector2f u = K * error_vector;
    double v_cmd = v_ref + u(0);
    double w_cmd = w_ref + u(1);

    return Velocity2d(current, v_cmd, w_cmd, 0.0);
}