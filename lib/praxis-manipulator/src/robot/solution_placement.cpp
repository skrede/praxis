#include "robot/solution_placement.h"

#include "praxis/manipulator/arm_snapshot.h"

#include <Eigen/Core>

#include <span>
#include <cmath>
#include <vector>
#include <limits>
#include <cstddef>
#include <numbers>
#include <optional>

namespace praxis::manipulator {

namespace {

// Radians, as the whole configuration's distance under the norm below.
constexpr double one_posture = 3.0e-3;

double wrapped(double difference)
{
    constexpr double turn = 2.0 * std::numbers::pi;

    double put = std::fmod(difference, turn);
    if(put > std::numbers::pi)
        put -= turn;
    else if(put <= -std::numbers::pi)
        put += turn;

    return put;
}

}

double joint_distance(const joint_vector &from, const joint_vector &to)
{
    if(from.size() != to.size())
        return std::numeric_limits<double>::infinity();

    double square = 0.0;
    for(Eigen::Index joint = 0; joint < from.size(); ++joint)
    {
        const double apart = wrapped(to[joint] - from[joint]);
        square += apart * apart;
    }

    return std::sqrt(square);
}

std::optional<std::size_t> nearest_solution(std::span<const joint_vector> among, const joint_vector &from)
{
    std::optional<std::size_t> nearest;
    double closest = std::numeric_limits<double>::infinity();
    for(std::size_t at = 0; at < among.size(); ++at)
    {
        if(among[at].size() != from.size())
            return std::nullopt;

        const double apart = joint_distance(from, among[at]);
        if(apart < closest)
        {
            closest = apart;
            nearest = at;
        }
    }

    return nearest;
}

std::optional<std::size_t> solution_at(const arm_snapshot &seen)
{
    return nearest_solution(seen.solutions, seen.joints);
}

std::size_t fold_solution(std::vector<joint_vector> &distinct, const joint_vector &found)
{
    for(std::size_t at = 0; at < distinct.size(); ++at)
        if(joint_distance(distinct[at], found) < one_posture)
            return at;

    distinct.push_back(found);

    return distinct.size() - 1u;
}

}
