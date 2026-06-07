#pragma once
#define EIGEN_DONT_VECTORIZE
#include <Eigen/Dense>

#include <array>
#include <vector>
#include <functional>
#include <algorithm>
#include <cmath>

template <typename T>
struct Source
{
    T *source;
    double weight;
};

class HeadingSource
{
public:
    virtual double thetaGains(double lastHeading, bool degrees = true);
    virtual void calibrate();
    virtual void tick() {};
    virtual ~HeadingSource() = default;
    virtual void reset() {};
};

class PositionSource
{
public:
    std::array<double, 2> gains(double dHeading);
    virtual ~PositionSource() = default;
    virtual void tick() {};
    virtual void reset() {};
};

class OdometryFilter
{
public:
    virtual double filterHeading(const std::vector<Source<HeadingSource>> &sources) = 0;
    virtual std::array<double, 2> filterPosition(
        const std::vector<Source<PositionSource>> &sources,
        double dHeading) = 0;

    virtual ~OdometryFilter() = default;
};

class EKFOdometryFilter : public OdometryFilter
{
public:
    using StateVec = Eigen::Matrix<double, 3, 1>;
    using StateMat = Eigen::Matrix<double, 3, 3>;

    EKFOdometryFilter(const StateMat &Q, const StateMat &P0 = StateMat::Identity())
        : Q_(Q), x_(StateVec::Zero()), P_(P0) {}

    void init(const StateVec &x0, const StateMat &P0 = StateMat::Identity())
    {
        x_ = x0;
        P_ = P0;
    }

    double filterHeading(const std::vector<Source<HeadingSource>> &sources) override
    {
        updateHeadingFromSources(sources);
        return x_(2);
    }

    std::array<double, 2> filterPosition(
        const std::vector<Source<PositionSource>> &sources,
        double dHeading) override
    {
        predict(0.0, 0.0, dHeading);
        updatePositionFromSources(sources, dHeading);
        return {x_(0), x_(1)};
    }

    void updateWithMeasurement(const StateVec &z, const StateMat &R_ext)
    {
        const StateVec y = z - x_;
        const StateMat S = P_ + R_ext;
        const StateMat K = P_ * S.inverse();
        x_ = x_ + K * y;
        x_(2) = normalizeAngle(x_(2));
        P_ = (StateMat::Identity() - K) * P_;
    }

    StateVec state()    const { return x_; }
    StateMat covariance() const { return P_; }
    double   heading()  const { return x_(2); }

private:
    void updatePositionFromSources(
        const std::vector<Source<PositionSource>> &sources,
        double dHeading)
    {
        if (sources.empty()) return;
        int m = (int)sources.size() * 2;

        Eigen::VectorXd z(m);
        Eigen::MatrixXd H(m, 3);
        Eigen::MatrixXd R(m, m);
        H.setZero(); R.setZero();

        for (int i = 0; i < (int)sources.size(); i++)
        {
            auto g = sources[i].source->gains(dHeading);
            z(i*2+0) = g[0];
            z(i*2+1) = g[1];
            H(i*2+0, 0) = 1.0;
            H(i*2+1, 1) = 1.0;
            double variance = (sources[i].weight > 1e-9) ? (1.0 / sources[i].weight) : 1e9;
            R(i*2+0, i*2+0) = variance;
            R(i*2+1, i*2+1) = variance;
        }
        kalmanUpdate(z, H, R);
    }

    void updateHeadingFromSources(const std::vector<Source<HeadingSource>> &sources)
    {
        if (sources.empty()) return;
        int m = (int)sources.size();

        Eigen::VectorXd z(m);
        Eigen::MatrixXd H(m, 3);
        Eigen::MatrixXd R(m, m);
        H.setZero(); R.setZero();

        for (int i = 0; i < m; i++)
        {
            z(i) = sources[i].source->thetaGains(x_(2), false);
            H(i, 2) = 1.0;
            double variance = (sources[i].weight > 1e-9) ? (1.0 / sources[i].weight) : 1e9;
            R(i, i) = variance;
        }

        Eigen::VectorXd y = z - H * x_;
        for (int i = 0; i < m; i++)
            y(i) = normalizeAngle(y(i));
        kalmanUpdateWithInnovation(y, H, R);
    }

    void kalmanUpdateWithInnovation(
        const Eigen::VectorXd &y,
        const Eigen::MatrixXd &H,
        const Eigen::MatrixXd &R)
    {
        Eigen::MatrixXd Ht = H.transpose();
        Eigen::MatrixXd S  = H * P_ * Ht + R;
        Eigen::MatrixXd K  = P_ * Ht * S.inverse();
        x_ = x_ + K * y;
        x_(2) = normalizeAngle(x_(2));
        P_ = (StateMat::Identity() - K * H) * P_;
    }

    void kalmanUpdate(
        const Eigen::VectorXd &z,
        const Eigen::MatrixXd &H,
        const Eigen::MatrixXd &R)
    {
        kalmanUpdateWithInnovation(z - H * x_, H, R);
    }

    void predict(double dX, double dY, double dTheta)
    {
        x_(0) += dX;
        x_(1) += dY;
        x_(2) = normalizeAngle(x_(2) + dTheta);
        P_ = P_ + Q_;
    }

    static double normalizeAngle(double a)
    {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    StateMat Q_;
    StateVec x_;
    StateMat P_;
};

// Declaration only — defined in filters.cpp
std::vector<double> extendedWeightedAverage(std::vector<std::vector<std::array<double, 2>>> extendedSums);