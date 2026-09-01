#include "two_pose_stage.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/screw.h"

#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <vector>
#include <cstddef>
#include <utility>

using namespace praxis;
using namespace praxis::rigid_motion;

namespace {

// Answers the screw of the reversed displacement, which is a different screw for every pair the
// suite passes it.
expected<std::pair<screw_axis, double>, refusal> reversed_screw(const transform &tf)
{
    return matrix_logarithm_se3(inverse(tf));
}

expected<std::pair<screw_axis, double>, refusal> refusing_screw(const transform &)
{
    return praxis::unexpected(refusal::degenerate);
}

transform turned_further(const screw_axis &s, double theta)
{
    return matrix_exponential_screw(s, theta + 1.0);
}

expected<std::pair<Eigen::Vector3d, double>, refusal> reversed_turn(const rotation &r)
{
    return matrix_logarithm_so3(rotation(r.transpose()));
}

}

TEST_CASE("the_moving_object_reaches_the_start_pose_at_zero_and_the_end_pose_at_one")
{
    fixture::staged resting(baseline(), fixture::at_parameter(0.f));
    fixture::staged midway(baseline(), fixture::at_parameter(0.5f));
    fixture::staged arrived(baseline(), fixture::at_parameter(1.f));

    resting.open();
    midway.open();
    arrived.open();

    CHECK(is_approx_equal(resting.placed(), resting.panel.start_pose(), 1.0e-9));
    CHECK(is_approx_equal(arrived.placed(), arrived.panel.end_pose(), 1.0e-9));

    // In between, the moving object stands at the sample of the drawn path that shares its
    // parameter: the placement and the path are the one expression.
    const std::vector<transform> path = midway.travelled;

    REQUIRE(path.size() == two_pose_window::path_points);
    CHECK(is_approx_equal(midway.placed(), path[(two_pose_window::path_points - 1) / 2], 1.0e-9));
    CHECK(is_approx_equal(path.front(), midway.panel.start_pose(), 1.0e-9));
    CHECK(is_approx_equal(path.back(), midway.panel.end_pose(), 1.0e-9));
}

TEST_CASE("a_substituted_exponential_carries_the_body_where_it_says_and_takes_the_path_with_it")
{
    capabilities exponent                   = baseline();
    exponent.screw.matrix_exponential_screw = &turned_further;

    fixture::staged reference(baseline(), fixture::at_parameter(0.5f));
    fixture::staged driven(exponent, fixture::at_parameter(0.5f));

    reference.open();
    driven.open();

    CHECK_FALSE(is_approx_equal(driven.placed(), reference.placed()));
    CHECK_FALSE(fixture::alike(driven.travelled, reference.travelled));

    // What the substituted exponential answers is where the body goes, rather than the window
    // arriving at the same place by an expression of its own.
    const transform start                                        = reference.panel.start_pose();
    const expected<std::pair<screw_axis, double>, refusal> named = matrix_logarithm_se3(reference.panel.end_pose() * inverse(start));

    REQUIRE(named.has_value());
    CHECK(is_approx_equal(driven.placed(), turned_further(named->first, 0.5 * named->second) * start, 1.0e-9));
}

TEST_CASE("a_substituted_logarithm_names_a_different_screw_and_the_body_and_the_path_both_follow_it")
{
    capabilities logarithm               = baseline();
    logarithm.screw.matrix_logarithm_se3 = &reversed_screw;

    fixture::staged reference(baseline(), fixture::at_parameter(0.5f));
    fixture::staged derived(logarithm, fixture::at_parameter(0.5f));

    reference.open();
    derived.open();

    CHECK_FALSE(is_approx_equal(derived.placed(), reference.placed()));
    CHECK_FALSE(fixture::alike(derived.travelled, reference.travelled));

    // Both ends still land where the controls say, because the substitution is a different route
    // between the same two poses rather than a different pair.
    CHECK(is_approx_equal(derived.travelled.front(), derived.panel.start_pose(), 1.0e-9));
}

TEST_CASE("two_equal_poses_name_no_motion_and_draw_a_path_of_no_extent")
{
    fixture::staged resting(baseline(), fixture::twice(fixture::opening().start));
    fixture::staged midway(baseline(), fixture::twice(fixture::opening().start));
    fixture::staged arrived(baseline(), fixture::twice(fixture::opening().start));

    resting.open();
    midway.open();
    arrived.open();

    REQUIRE(resting.invocations == 1u);
    CHECK(resting.travelled.empty());
    CHECK(resting.decoupled_travelled.empty());
    CHECK_FALSE(resting.panel.reading().message.empty());
    CHECK(resting.panel.reading().rows.empty());
    CHECK_FALSE(resting.decoupled.reading().message.empty());
    CHECK(resting.decoupled.reading().rows.empty());

    // The moving object is at the start pose whatever the parameter says, because there is nowhere
    // else for it to be.
    CHECK(is_approx_equal(resting.placed(), resting.panel.start_pose()));
    CHECK(is_approx_equal(midway.placed(), midway.panel.start_pose()));
    CHECK(is_approx_equal(arrived.placed(), arrived.panel.start_pose()));
}

// What stands between two equal poses and a path drawn along an axis they never named is the guard
// and not the operations: neither logarithm refuses an identity, and what each answers is a
// confident unit direction, so nothing downstream would ever report that the direction was
// arbitrary. This is asserted here so that the guard cannot be removed on a wrong theory about it.
TEST_CASE("neither_logarithm_refuses_an_identity_and_each_answers_a_unit_direction_it_was_never_given")
{
    const expected<std::pair<screw_axis, double>, refusal> named       = matrix_logarithm_se3(transform::Identity());
    const expected<std::pair<Eigen::Vector3d, double>, refusal> turned = matrix_logarithm_so3(rotation::Identity());

    REQUIRE(named.has_value());
    REQUIRE(turned.has_value());
    CHECK(is_approx_equal(named->first.norm(), 1.0, 1.0e-12));
    CHECK(named->first.head<3>().isZero());
    CHECK(is_approx_equal(named->second, 0.0));
    CHECK(is_approx_equal(turned->first.norm(), 1.0, 1.0e-12));
    CHECK(is_approx_equal(turned->second, 0.0));
}

TEST_CASE("a_refused_logarithm_leaves_the_moving_object_where_it_was_and_draws_nothing")
{
    capabilities refusing               = baseline();
    refusing.screw.matrix_logarithm_se3 = &refusing_screw;

    fixture::staged resting(refusing, fixture::at_parameter(0.f));
    fixture::staged arrived(refusing, fixture::at_parameter(1.f));
    const transform before = resting.placed();

    resting.open();
    arrived.open();

    REQUIRE(resting.invocations == 1u);
    CHECK(resting.travelled.empty());
    CHECK_FALSE(resting.panel.reading().message.empty());
    CHECK(is_approx_equal(resting.placed(), before));
    CHECK(is_approx_equal(arrived.placed(), before));

    // The two poses the controls name are not a guess and are placed anyway, so what the picture
    // shows is two markers and no body between them.
    CHECK(is_approx_equal(resting.body.pose(two_pose_window::start_object), resting.panel.start_pose()));
    CHECK(is_approx_equal(resting.body.pose(two_pose_window::end_object), resting.panel.end_pose()));
}

TEST_CASE("the_path_route_runs_where_a_pose_changed_and_on_no_frame_where_neither_did")
{
    fixture::staged built(baseline());
    built.open();

    REQUIRE(built.invocations == 1u);

    built.idle(3);

    CHECK(built.invocations == 1u);

    const std::size_t offered = built.controls();

    REQUIRE(offered > 0u);

    const std::size_t moved = fixture::controls_moving_a_pose(built, offered);

    // Each pose is named by three angles, three positions and the axis order the angles are taken
    // in; the fifteenth control is the parameter, which carries the moving object along a path it
    // does not ask for again.
    CHECK(moved == 14u);
    CHECK(offered == moved + 1u);
}

TEST_CASE("the_two_paths_coincide_for_a_slide_and_part_for_a_turn_off_the_line_between_the_poses")
{
    fixture::staged slid(baseline(), fixture::slide_only());
    fixture::staged turned(baseline());

    slid.open();
    turned.open();

    REQUIRE(slid.travelled.size() == two_pose_window::path_points);
    REQUIRE(slid.decoupled_travelled.size() == two_pose_window::path_points);
    CHECK(fixture::alike(slid.travelled, slid.decoupled_travelled));
    CHECK_FALSE(fixture::alike(turned.travelled, turned.decoupled_travelled));

    // They part in between and meet again at both ends, because both are the same two poses carried
    // between.
    CHECK(is_approx_equal(turned.travelled.front(), turned.decoupled_travelled.front(), 1.0e-9));
    CHECK(is_approx_equal(turned.travelled.back(), turned.decoupled_travelled.back(), 1.0e-9));
    CHECK(fixture::apart(turned.travelled, turned.decoupled_travelled) > 0.1);
}

TEST_CASE("each_path_follows_the_operations_it_is_drawn_from_and_leaves_the_other_path_alone")
{
    capabilities rigid               = baseline();
    rigid.screw.matrix_logarithm_se3 = &reversed_screw;

    capabilities turning               = baseline();
    turning.screw.matrix_logarithm_so3 = &reversed_turn;

    fixture::staged reference(baseline());
    fixture::staged displaced(rigid);
    fixture::staged rotated(turning);

    reference.open();
    displaced.open();
    rotated.open();

    CHECK_FALSE(fixture::alike(displaced.travelled, reference.travelled));
    CHECK(fixture::alike(displaced.decoupled_travelled, reference.decoupled_travelled));

    CHECK(fixture::alike(rotated.travelled, reference.travelled));
    CHECK_FALSE(fixture::alike(rotated.decoupled_travelled, reference.decoupled_travelled));
}
