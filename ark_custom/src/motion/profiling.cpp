#include "../ark_custom/include/motion/profiling.h"
#include <vector>
#include <array>
#include <cmath>
#include <stdexcept>

namespace MotionProfile
{

    double QuinticHermite::eval(double t) const
    {
        double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
        return p0 * (1 - 10 * t3 + 15 * t4 - 6 * t5) + v0 * (t - 6 * t3 + 8 * t4 - 3 * t5) + a0 * (0.5 * t2 - 1.5 * t3 + 1.5 * t4 - 0.5 * t5) + p1 * (10 * t3 - 15 * t4 + 6 * t5) + v1 * (-4 * t3 + 7 * t4 - 3 * t5) + a1 * (0.5 * t3 - t4 + 0.5 * t5);
    }

    double QuinticHermite::evalDerivative(double t) const
    {
        double t2 = t * t, t3 = t2 * t, t4 = t3 * t;
        return p0 * (-30 * t2 + 60 * t3 - 30 * t4) + v0 * (1 - 18 * t2 + 32 * t3 - 15 * t4) + a0 * (t - 4.5 * t2 + 6 * t3 - 2.5 * t4) + p1 * (30 * t2 - 60 * t3 + 30 * t4) + v1 * (-12 * t2 + 28 * t3 - 15 * t4) + a1 * (1.5 * t2 - 4 * t3 + 2.5 * t4);
    }

    std::vector<Velocity2d> resample(
        const std::vector<Velocity2d> &pts,
        double minDist)
    {
        if (pts.empty())
            return {};
        std::vector<Velocity2d> out;
        out.push_back(pts.front());

        for (size_t i = 1; i < pts.size(); i++)
        {
            if (out.back().getDistance(pts[i]) >= minDist)
                out.push_back(pts[i]);
        }

        if (out.back() != pts.back())
            out.push_back(pts.back());

        return out;
    }

    double curvatureVelocity(double vMax, double curvature, double k)
    {
        return vMax / (1.0 + k * std::abs(curvature));
    }

    std::vector<Velocity2d> linear(
        const Pose2d &start,
        const Pose2d &end,
        double vMax,
        int samples,
        double minDist)
    {
        std::vector<Velocity2d> pts;
        pts.reserve(samples + 1);

        double dx = end.getX() - start.getX();
        double dy = end.getY() - start.getY();
        double heading = std::atan2(dy, dx);

        for (int i = 0; i <= samples; i++)
        {
            double t = (double)i / samples;
            pts.emplace_back(
                start.getX() + t * dx,
                start.getY() + t * dy,
                heading,
                vMax, 0.0, 0.0);
        }

        return minDist > 0.0 ? resample(pts, minDist) : pts;
    }

    Pose2d bezierPoint(const BezierSegment &seg, double t)
    {
        double u = 1.0 - t;
        double u2 = u * u, u3 = u2 * u;
        double t2 = t * t, t3 = t2 * t;

        double x = u3 * seg.p0.getX() + 3 * u2 * t * seg.p1.getX() + 3 * u * t2 * seg.p2.getX() + t3 * seg.p3.getX();
        double y = u3 * seg.p0.getY() + 3 * u2 * t * seg.p1.getY() + 3 * u * t2 * seg.p2.getY() + t3 * seg.p3.getY();

        double dx = 3 * u2 * (seg.p1.getX() - seg.p0.getX()) + 6 * u * t * (seg.p2.getX() - seg.p1.getX()) + 3 * t2 * (seg.p3.getX() - seg.p2.getX());
        double dy = 3 * u2 * (seg.p1.getY() - seg.p0.getY()) + 6 * u * t * (seg.p2.getY() - seg.p1.getY()) + 3 * t2 * (seg.p3.getY() - seg.p2.getY());

        return Pose2d(x, y, std::atan2(dy, dx));
    }

    double bezierCurvature(const BezierSegment &seg, double t)
    {
        double u = 1.0 - t;

        double dx = 3 * u * u * (seg.p1.getX() - seg.p0.getX()) + 6 * u * t * (seg.p2.getX() - seg.p1.getX()) + 3 * t * t * (seg.p3.getX() - seg.p2.getX());
        double dy = 3 * u * u * (seg.p1.getY() - seg.p0.getY()) + 6 * u * t * (seg.p2.getY() - seg.p1.getY()) + 3 * t * t * (seg.p3.getY() - seg.p2.getY());

        double ddx = 6 * u * (seg.p2.getX() - 2 * seg.p1.getX() + seg.p0.getX()) + 6 * t * (seg.p3.getX() - 2 * seg.p2.getX() + seg.p1.getX());
        double ddy = 6 * u * (seg.p2.getY() - 2 * seg.p1.getY() + seg.p0.getY()) + 6 * t * (seg.p3.getY() - 2 * seg.p2.getY() + seg.p1.getY());

        double denom = std::pow(dx * dx + dy * dy, 1.5);
        if (std::abs(denom) < 1e-9)
            return 0.0;
        return (dx * ddy - dy * ddx) / denom;
    }

    std::vector<Velocity2d> bezierSpline(
        const std::vector<BezierSegment> &segments,
        double vMax,
        double curvatureGain,
        int samplesPerSegment,
        double minDist)
    {
        std::vector<Velocity2d> pts;

        for (const auto &seg : segments)
        {
            for (int i = 0; i <= samplesPerSegment; i++)
            {
                double t = (double)i / samplesPerSegment;
                Pose2d p = bezierPoint(seg, t);
                double kappa = bezierCurvature(seg, t);
                double v = curvatureVelocity(vMax, kappa, curvatureGain);
                double omega = v * kappa;
                pts.emplace_back(p, v, omega, 0.0);
            }
        }

        return minDist > 0.0 ? resample(pts, minDist) : pts;
    }
    
    std::vector<Velocity2d> quinticHermiteSpline(
        const std::vector<HermiteWaypoint> &waypoints,
        double vMax,
        double curvatureGain,
        int samplesPerSegment,
        double minDist)
    {
        std::vector<Velocity2d> pts;

        for (size_t w = 0; w + 1 < waypoints.size(); w++)
        {
            const auto &wp0 = waypoints[w];
            const auto &wp1 = waypoints[w + 1];

            double dist = wp0.pose.getDistance(wp1.pose);
            double scale0 = dist * wp0.speed;
            double scale1 = dist * wp1.speed;

            double cos0 = std::cos(wp0.pose.getTheta()), sin0 = std::sin(wp0.pose.getTheta());
            double cos1 = std::cos(wp1.pose.getTheta()), sin1 = std::sin(wp1.pose.getTheta());

            QuinticHermite hx{
                wp0.pose.getX(), scale0 * cos0, wp0.curvature * scale0 * scale0 * (-sin0),
                wp1.pose.getX(), scale1 * cos1, wp1.curvature * scale1 * scale1 * (-sin1)};
            QuinticHermite hy{
                wp0.pose.getY(), scale0 * sin0, wp0.curvature * scale0 * scale0 * cos0,
                wp1.pose.getY(), scale1 * sin1, wp1.curvature * scale1 * scale1 * cos1};

            for (int i = 0; i <= samplesPerSegment; i++)
            {
                double t = (double)i / samplesPerSegment;
                double x = hx.eval(t);
                double y = hy.eval(t);
                double dx = hx.evalDerivative(t);
                double dy = hy.evalDerivative(t);

                double heading = std::atan2(dy, dx);
                double speed2 = dx * dx + dy * dy;
                double ds = std::sqrt(speed2);

                double ddx = hx.evalDerivative(t + 1e-5) - dx;
                double ddy = hy.evalDerivative(t + 1e-5) - dy;
                double kappa = (std::abs(ds) > 1e-9)
                                   ? (dx * ddy - dy * ddx) / (speed2 * ds * 1e-5)
                                   : 0.0;

                double v = curvatureVelocity(vMax, kappa, curvatureGain);
                double omega = v * kappa;

                pts.emplace_back(x, y, heading, v, omega, 0.0);
            }
        }

        return minDist > 0.0 ? resample(pts, minDist) : pts;
    }

}