#include "answered.h"

#include "praxis/trajectory/path.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <type_traits>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::trajectory;

static_assert(std::is_aggregate_v<path_ops>);
static_assert(std::is_trivially_copyable_v<path_ops>);

namespace {

expected<configuration, refusal> straight_line(const configuration &start, const configuration &end, double s)
{
    return configuration(start + s * (end - start));
}

// Not the full decoupled path -- its rotation is held at the start rather than interpolated,
// because the matrix logarithm and exponential the real one needs are themselves slots and hold
// their inert defaults here. What it does carry is the property that separates the two task-space
// paths: the origin travels the straight line between the endpoints.
expected<transform, refusal> straight_line_origin(const transform &start, const transform &end, double s)
{
    transform pose         = start;
    pose.block<3, 1>(0, 3) = start.block<3, 1>(0, 3) + s * (end.block<3, 1>(0, 3) - start.block<3, 1>(0, 3));
    return pose;
}

transform endpoints()
{
    transform pose         = transform::Identity();
    pose.block<3, 1>(0, 3) = Eigen::Vector3d(2.0, 4.0, 6.0);
    return pose;
}

}

TEST_CASE("the_path_slots_default_to_refusing_rather_than_to_a_pose_a_bound_path_could_have_produced")
{
    path_ops ops{};
    configuration start = configuration::Constant(3, 0.25);
    configuration end   = configuration::Constant(3, 1.25);

    REQUIRE(ops.joint_straight_line != nullptr);
    REQUIRE(ops.screw != nullptr);
    REQUIRE(ops.decoupled != nullptr);

    CHECK(ops.joint_straight_line(start, end, 0.5).error() == refusal::not_implemented);
    CHECK(ops.screw(transform::Identity(), endpoints(), 0.5).error() == refusal::not_implemented);
    CHECK(ops.decoupled(transform::Identity(), endpoints(), 0.5).error() == refusal::not_implemented);
}

TEST_CASE("a_configuration_space_path_bound_into_the_seam_meets_the_endpoint_conditions")
{
    path_ops ops{.joint_straight_line = &straight_line};
    configuration start = configuration::Constant(3, 0.25);
    configuration end   = configuration::Constant(3, 1.25);

    REQUIRE(is_approx_equal(answered(ops.joint_straight_line(start, end, 0.0)), start));
    REQUIRE(is_approx_equal(answered(ops.joint_straight_line(start, end, 1.0)), end));
    REQUIRE(is_approx_equal(answered(ops.joint_straight_line(start, end, 0.5)), configuration::Constant(3, 0.75)));
}

TEST_CASE("the_two_task_space_path_slots_are_independently_bound_and_only_the_bound_one_carries_a_path")
{
    path_ops ops{.decoupled = &straight_line_origin};
    path_ops defaults{};
    transform end = endpoints();

    REQUIRE(ops.screw == defaults.screw);
    REQUIRE(ops.joint_straight_line == defaults.joint_straight_line);

    REQUIRE(is_approx_equal(answered(ops.decoupled(transform::Identity(), end, 0.0)), transform::Identity()));
    REQUIRE(is_approx_equal(answered(ops.decoupled(transform::Identity(), end, 1.0)), end));
    REQUIRE(answered(ops.decoupled(transform::Identity(), end, 0.5)).block<3, 1>(0, 3).isApprox(Eigen::Vector3d(1.0, 2.0, 3.0), default_tolerance));
    REQUIRE(!ops.screw(transform::Identity(), end, 0.5));
}
