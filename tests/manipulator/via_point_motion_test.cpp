#include "fixtures.h"

#include "praxis/manipulator/motion_commands.h"

#include "praxis/trajectory/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

constexpr std::uint32_t sampled_intervals = 64;

std::size_t slot_asks = 0;

joint_limits two_joint_limits()
{
    joint_limits bounds{};
    bounds.velocity       = joint_vector::Constant(2, 1.0);
    bounds.acceleration   = joint_vector::Constant(2, 4.0);
    bounds.lower_position = joint_vector::Constant(2, -1.5);
    bounds.upper_position = joint_vector::Constant(2, 1.5);

    return bounds;
}

expected<std::vector<trajectory::via_point>, refusal> refusing_schedule(std::span<const joint_vector>, const joint_vector &, const joint_limits &)
{
    return praxis::unexpected(refusal::unsupported_input);
}

expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> counted_refusing_slot(std::span<const trajectory::via_point>)
{
    ++slot_asks;

    return praxis::unexpected(refusal::degenerate);
}

bool same_sample(const trajectory::trajectory_generator &left, const trajectory::trajectory_generator &right, double t)
{
    const expected<trajectory::trajectory_sample, refusal> a = left.sample(t);
    const expected<trajectory::trajectory_sample, refusal> b = right.sample(t);
    if(!a || !b)
        return false;

    return a->position == b->position && a->velocity == b->velocity && a->acceleration == b->acceleration;
}

}

TEST_CASE("a via-point motion is the step's schedule joined by the via-point slot of the operations handed in", "[via_point_motion]")
{
    const trajectory::trajectory_ops reference = trajectory::baseline().trajectory;
    const std::array<joint_vector, 3> positions{configuration(0.4, -0.2), configuration(0.9, 0.3), configuration(0.1, 0.6)};
    const joint_vector j0 = configuration(0.0, 0.0);

    const auto composed = via_point_motion(reference, positions, j0, two_joint_limits());
    const auto schedule = reference.via_point_schedule(positions, j0, two_joint_limits());
    REQUIRE(composed);
    REQUIRE(schedule);
    const auto direct = reference.joint_space_via_points(*schedule);
    REQUIRE(direct);

    const double span = (*composed)->duration();
    CHECK(span == (*direct)->duration());
    for(std::uint32_t at = 0; at <= sampled_intervals; ++at)
        CHECK(same_sample(**composed, **direct, span * at / sampled_intervals));
}

TEST_CASE("a refused schedule is answered and the via-point slot is not asked", "[via_point_motion]")
{
    trajectory::trajectory_ops composed   = trajectory::baseline().trajectory;
    composed.via_point_schedule           = &refusing_schedule;
    composed.joint_space_via_points       = &counted_refusing_slot;
    slot_asks                             = 0;
    const std::array<joint_vector, 1> one = {configuration(0.9, -0.9)};

    const auto motion = via_point_motion(composed, one, configuration(0.0, 0.0), two_joint_limits());

    REQUIRE_FALSE(motion);
    CHECK(motion.error() == refusal::unsupported_input);
    CHECK(slot_asks == 0u);
}

TEST_CASE("a refusal of the via-point slot is answered", "[via_point_motion]")
{
    trajectory::trajectory_ops composed   = trajectory::baseline().trajectory;
    composed.joint_space_via_points       = &counted_refusing_slot;
    slot_asks                             = 0;
    const std::array<joint_vector, 1> one = {configuration(0.9, -0.9)};

    const auto motion = via_point_motion(composed, one, configuration(0.0, 0.0), two_joint_limits());

    REQUIRE_FALSE(motion);
    CHECK(motion.error() == refusal::degenerate);
    CHECK(slot_asks == 1u);
}
