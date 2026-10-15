#include "via_point_cubics.h"

#include "praxis/trajectory/baseline/trajectory.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <cstddef>

namespace praxis::trajectory {

namespace {

bool enough(std::span<const via_point> via_points)
{
    if(via_points.size() >= 2u)
        return true;

    spdlog::error("praxis: a run of via points joins at least two and {} were given, and the run stands as it was given", via_points.size());
    return false;
}

bool starts_at_zero(std::span<const via_point> via_points)
{
    if(via_points.front().time == 0.0)
        return true;

    spdlog::error("praxis: a run of via points starts at time 0 and the first stands at {}, and the run stands as it was given", via_points.front().time);
    return false;
}

bool times_advance(std::span<const via_point> via_points)
{
    for(std::size_t k = 1u; k < via_points.size(); ++k)
        if(!std::isfinite(via_points[k].time) || !(via_points[k].time > via_points[k - 1u].time))
        {
            spdlog::error("praxis: via point {} stands at time {}, which is not a finite time after the via point before it, and the run stands as it was given", k, via_points[k].time);
            return false;
        }

    return true;
}

bool widths_agree(std::span<const via_point> via_points)
{
    const Eigen::Index width = via_points.front().position.size();
    for(std::size_t k = 0u; k < via_points.size(); ++k)
        if(via_points[k].position.size() != width || via_points[k].velocity.size() != width)
        {
            spdlog::error("praxis: a run of via points holds one width, via point {} holds a position of {} and a velocity of {} where the first position holds {}, and the run "
                          "stands as it was given",
                          k, via_points[k].position.size(), via_points[k].velocity.size(), width);
            return false;
        }

    return true;
}

bool all_finite(std::span<const via_point> via_points)
{
    for(std::size_t k = 0u; k < via_points.size(); ++k)
        if(!via_points[k].position.allFinite() || !via_points[k].velocity.allFinite())
        {
            spdlog::error("praxis: via point {} holds a position or velocity that is not finite, and the run stands as it was given", k);
            return false;
        }

    return true;
}

}

// Lynch & Park, Modern Robotics, sec. 9.3, eq. (9.25)-(9.29).
expected<std::unique_ptr<trajectory_generator>, refusal> joint_space_via_points(std::span<const via_point> via_points)
{
    if(!enough(via_points) || !starts_at_zero(via_points) || !times_advance(via_points) || !widths_agree(via_points) || !all_finite(via_points))
        return unexpected(refusal::unsupported_input);

    return via_point_generator(run_through(via_points));
}

}
