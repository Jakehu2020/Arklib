#pragma once
#include "../util/pose.h"
#include <vector>
#include <array>
#include <cmath>
#include <stdexcept>

namespace MotionProfile
{

    std::vector<Velocity2d> resample(
        const std::vector<Velocity2d> &pts,
        double minDist);

    double curvatureVelocity(double vMax, double curvature, double k = 1.0);

    std::vector<Velocity2d> linear(
        const Pose2d &start,
        const Pose2d &end,
        double vMax,
        int samples = 100,
        double minDist = 0.0);

    struct BezierSegment
    {
        Pose2d p0, p1, p2, p3;
    };

    Pose2d bezierPoint(const BezierSegment &seg, double t);
    double bezierCurvature(const BezierSegment &seg, double t);

    std::vector<Velocity2d> bezierSpline(
        const std::vector<BezierSegment> &segments,
        double vMax,
        double curvatureGain = 1.0,
        int samplesPerSegment = 100,
        double minDist = 0.0);

    // C²-Continuous Quintic Hermite Spline
    struct HermiteWaypoint
    {
        Pose2d pose;
        double curvature = 0.0; // d(heading)/ds at this point
        double speed = 1.0;     // tangent magnitude scaling at this knot
    };

    struct QuinticHermite
    {
        double p0, v0, a0, p1, v1, a1;

        double eval(double t) const;
        double evalDerivative(double t) const;
    };

    std::vector<Velocity2d> quinticHermiteSpline(
        const std::vector<HermiteWaypoint> &waypoints,
        double vMax,
        double curvatureGain = 1.0,
        int samplesPerSegment = 100,
        double minDist = 0.0);
}