#pragma once
#include "v5.h"

// Wheel diameters in inches. These values were measured by Lemlib.
// https://lemlib.readthedocs.io/en/stable/tutorials/2_configuration.html
// "_HALF" refers to half-cut wheels.

struct Wheel {
    static constexpr double NEW_2 = 2.125;
    static constexpr double NEW_275 = 2.75;
    static constexpr double OLD_275 = 2.75;
    static constexpr double NEW_275_HALF = 2.744;
    static constexpr double OLD_275_HALF = 2.74;
    static constexpr double NEW_325 = 3.25;
    static constexpr double OLD_325 = 3.25;
    static constexpr double NEW_325_HALF = 3.246;
    static constexpr double OLD_325_HALF = 3.246;
    static constexpr double NEW_4 = 4.00;
    static constexpr double OLD_4 = 4.18;
    static constexpr double NEW_4_HALF = 3.995;
    static constexpr double OLD_4_HALF = 4.175;
};

struct Distance {
    static constexpr double INCH = 1.0;
    static constexpr double FOOT = 12.0;
    static constexpr double TILE = 24.0;
    static constexpr double MM = 0.0393701;
    static constexpr double CM = 0.393701;
};