#include "../ark_custom/include/util/filters.h"
#include <algorithm>
#include <functional>

std::vector<double> extendedWeightedAverage(
    std::vector<std::vector<std::array<double, 2>>> extendedSums)
{
    std::vector<double> sums;
    std::vector<double> totalWeight;

    for (const std::vector<std::array<double, 2>> &values : extendedSums)
    {
        double sum = 0.0, weight = 0.0;
        for (const std::array<double, 2> &value : values)
        {
            sum    += value[0] * value[1];
            weight += value[1];
        }
        sums.push_back(sum);
        totalWeight.push_back(weight);
    }

    std::vector<double> result;
    std::transform(
        sums.begin(), sums.end(),
        totalWeight.begin(),
        std::back_inserter(result),
        std::divides<double>());

    return result;
}