#include "evaluation_cases.h"

#include "praxis/manipulator/motion_commands.h"

#include "praxis/trajectory/capabilities.h"

#include <catch2/catch_test_macros.hpp>

using namespace praxis;
using namespace praxis::manipulator;

namespace {

// One joint at unit limits; held bounds leave the duration independent of both.
double handed_duration(path_parameter_bounds held)
{
    joint_limits limits{};
    limits.velocity     = joint_vector::Constant(1, 1.0);
    limits.acceleration = joint_vector::Constant(1, 1.0);
    const prepared_time_scaling scaled(trajectory::baseline().time_scaling, time_scaling_choice::trapezoidal, held);

    return scaled.duration(limits, joint_vector::Constant(1, 0.0), joint_vector::Constant(1, 1.0));
}

}

TEST_CASE("the_shortest_duration_the_trapezoidal_row_draws_is_the_one_the_scene_hands_for_the_same_bounds")
{
    for(const path_parameter_bounds held : {path_parameter_bounds{0.5, 2.0}, path_parameter_bounds{2.0, 0.5}, path_parameter_bounds{1.0, 1.0}})
    {
        INFO("bounds " << held.max_rate << ", " << held.max_rate_change);
        REQUIRE(trajectory::trapezoid_duration_floor(held.max_rate, held.max_rate_change) == handed_duration(held));
    }
}
