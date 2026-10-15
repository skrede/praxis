#include "evaluation_cases.h"
#include "configuration_run.h"

#include "praxis/trajectory/baseline/trajectory.h"

#include "praxis/manipulator/edited_list_rows.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/tolerance.h"
#include "praxis/evaluation/generation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <vector>
#include <cstddef>
#include <numbers>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::trajectory;

namespace {

constexpr double exact_tolerance       = 1.0e-12;
constexpr std::size_t reproduced_cases = 2000u;
constexpr std::size_t sampled_instants = 64u;

joint_waypoint_case drawn_run(std::size_t index)
{
    evaluation::case_source drawn = evaluation::case_source::at_case(evaluation::default_seed, "trajectory.via_point_schedule", evaluation::spread::bulk, index);

    return drawn_joint_waypoint_case(drawn);
}

bool identical(const trajectory_sample &first, const trajectory_sample &second)
{
    return first.position == second.position && first.velocity == second.velocity && first.acceleration == second.acceleration;
}

bool reproduces(std::span<const configuration> positions, const configuration &j0, const configuration_limits &limits)
{
    const auto today = joint_space_waypoints(positions, j0, limits);
    const auto run   = joined(scheduled(positions, j0, limits));
    REQUIRE(today);

    const double span = (*today)->duration();
    if(run->duration() != span)
        return false;
    for(std::size_t k = 0u; k <= sampled_instants; ++k)
    {
        const double t = span * static_cast<double>(k) / static_cast<double>(sampled_instants);
        if(!identical(at(**today, t), at(*run, t)))
            return false;
    }

    return true;
}

}

TEST_CASE("the_schedule_joined_by_the_via_point_slot_is_every_drawn_run_of_three_or_more_bit_for_bit")
{
    std::size_t compared = 0u;
    for(std::size_t index = 0u; compared < reproduced_cases; ++index)
    {
        const joint_waypoint_case run = drawn_run(index);
        if(run.waypoints.size() < 3u)
            continue;

        INFO("case " << index);
        REQUIRE(reproduces(run.waypoints, run.seed, run.limits));
        ++compared;
    }
}

TEST_CASE("the_schedule_joined_by_the_via_point_slot_is_the_opening_configuration_list_bit_for_bit")
{
    constexpr double degrees                 = std::numbers::pi / 180.0;
    const std::vector<configuration> opening = manipulator::list_row_traits<manipulator::joint_vector>::opening_rows(6u);
    configuration_limits limits{};
    limits.velocity = configuration(6);
    limits.velocity << 360.0 * degrees, 300.0 * degrees, 360.0 * degrees, 381.0 * degrees, 388.0 * degrees, 615.0 * degrees;

    REQUIRE(opening.size() == 4u);
    REQUIRE(reproduces(opening, configuration::Zero(6), limits));
}

TEST_CASE("a_schedule_of_three_or_more_starts_at_rest_at_zero_ends_at_rest_and_passes_each_interior_row_at_the_book_velocity")
{
    for(std::size_t index = 0u; index < reproduced_cases; ++index)
    {
        const joint_waypoint_case run = drawn_run(index);
        if(run.waypoints.size() < 3u)
            continue;

        const std::vector<via_point> via = scheduled(run.waypoints, run.seed, run.limits);
        std::vector<double> times;
        for(const via_point &point : via)
            times.push_back(point.time);

        INFO("case " << index);
        REQUIRE(via.front().time == 0.0);
        REQUIRE(std::adjacent_find(times.begin(), times.end(), [](double before, double after) { return !(after > before); }) == times.end());
        REQUIRE(via.front().velocity.isZero(0.0));
        REQUIRE(via.back().velocity.isZero(0.0));
        for(std::size_t k = 1u; k + 1u < via.size(); ++k)
            REQUIRE(is_approx_equal(via[k].velocity, book_velocity(run.waypoints, times, k), exact_tolerance));
    }
}
