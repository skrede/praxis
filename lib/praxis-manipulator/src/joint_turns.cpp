#include "joint_turns.h"

#include "praxis/evaluation/tolerance.h"

#include <cmath>
#include <limits>
#include <numbers>
#include <cstddef>
#include <algorithm>

namespace praxis::manipulator {

namespace {

// In machine epsilons of the largest finite operand's magnitude.
constexpr double period_count_slack = 2.0;

std::optional<double> nearest_within(double value, double near, double lower, double upper, double period)
{
    if(period == 0.0)
        return lower <= value && value <= upper ? std::optional<double>(value) : std::nullopt;

    double magnitude = 1.0;
    for(const double operand : {value, near, lower, upper})
        if(std::isfinite(operand))
            magnitude = std::max(magnitude, std::abs(operand));

    const double slack  = period_count_slack * std::numeric_limits<double>::epsilon() * magnitude / period;
    const double fewest = std::ceil((lower - value) / period - slack);
    const double most   = std::floor((upper - value) / period + slack);
    if(fewest > most)
        return std::nullopt;

    const double turns = std::clamp(std::round((near - value) / period), fewest, most);

    return std::clamp(value + period * turns, lower, upper);
}

}

double whole_turn(const screw_axis &s)
{
    const bool rotation = is_approx_equal(s.head<3>().norm(), 1.0) && is_approx_equal(s.head<3>().dot(s.tail<3>()), 0.0);

    return rotation ? 2.0 * std::numbers::pi : 0.0;
}

std::optional<joint_vector> turned_within_limits(const screw_chain &chain, const joint_vector &q, const joint_vector &near)
{
    const auto width = static_cast<Eigen::Index>(chain.joint_count());
    if(q.size() != width || near.size() != width || chain.limits.lower_position.size() != width || chain.limits.upper_position.size() != width)
        return std::nullopt;

    joint_vector turned(width);
    for(Eigen::Index joint = 0; joint < width; ++joint)
    {
        const double period               = whole_turn(chain.space_screws[static_cast<std::size_t>(joint)]);
        const std::optional<double> named = nearest_within(q[joint], near[joint], chain.limits.lower_position[joint], chain.limits.upper_position[joint], period);
        if(!named)
            return std::nullopt;

        turned[joint] = *named;
    }

    return turned;
}

}
