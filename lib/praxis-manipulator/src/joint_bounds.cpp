#include "joint_bounds.h"

#include "praxis/evaluation/tolerance.h"

#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <algorithm>

namespace praxis::manipulator {

namespace {

// Lynch & Park, Modern Robotics, Def. 3.24: a unit screw of pitch w . v = 0 is a pure rotation, so a
// whole turn about it is no displacement.
bool turns_whole(const screw_axis &s)
{
    return is_approx_equal(s.head<3>().norm(), 1.0) && is_approx_equal(s.head<3>().dot(s.tail<3>()), 0.0);
}

// value + k turn inside [lower, upper], k nearest zero; a turn of zero admits the value alone.
std::optional<double> named_between(double value, double lower, double upper, double turn)
{
    if(turn == 0.0)
        return lower <= value && value <= upper ? std::optional<double>(value) : std::nullopt;

    const double fewest = std::ceil((lower - value) / turn);
    const double most   = std::floor((upper - value) / turn);
    if(fewest > most)
        return std::nullopt;

    return value + turn * std::clamp(0.0, fewest, most);
}

}

std::optional<joint_vector> named_inside_bounds(const screw_chain &chain, const joint_vector &candidate)
{
    const joint_limits &bounds = chain.limits;

    joint_vector named = candidate;
    for(Eigen::Index joint = 0; joint < named.size(); ++joint)
    {
        if(joint >= bounds.lower_position.size() || joint >= bounds.upper_position.size())
            continue;

        const double turn                  = turns_whole(chain.space_screws[static_cast<std::size_t>(joint)]) ? 2.0 * std::numbers::pi : 0.0;
        const std::optional<double> inside = named_between(named[joint], bounds.lower_position[joint], bounds.upper_position[joint], turn);
        if(!inside.has_value())
            return std::nullopt;

        named[joint] = *inside;
    }

    return named;
}

}
