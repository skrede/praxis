#include "periodic_bounds.h"

#include <cmath>
#include <limits>
#include <algorithm>

namespace praxis::optimization::projected {

namespace {

// In machine epsilons of the largest finite operand's magnitude.
constexpr double period_count_slack = 2.0;

}

std::optional<double> named_between(double value, double lower, double upper, double period)
{
    if(period == 0.0)
        return lower <= value && value <= upper ? std::optional<double>(value) : std::nullopt;

    double magnitude = 1.0;
    for(const double operand : {value, lower, upper})
        if(std::isfinite(operand))
            magnitude = std::max(magnitude, std::abs(operand));

    const double slack  = period_count_slack * std::numeric_limits<double>::epsilon() * magnitude / period;
    const double fewest = std::ceil((lower - value) / period - slack);
    const double most   = std::floor((upper - value) / period + slack);
    if(fewest > most)
        return std::nullopt;

    return std::min(std::max(value + period * std::clamp(0.0, fewest, most), lower), upper);
}

double projected_between(double value, double lower, double upper, double period)
{
    if(const std::optional<double> inside = named_between(value, lower, upper, period))
        return *inside;

    const double crossed = std::min(std::max(value, lower), upper);
    if(period == 0.0)
        return crossed;

    const double past_lower = std::abs(std::remainder(value - lower, period));
    const double past_upper = std::abs(std::remainder(value - upper, period));
    if(past_lower == past_upper)
        return crossed;

    return past_lower < past_upper ? lower : upper;
}

}
