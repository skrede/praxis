#include "opened_two_pose.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/screw.h"

#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::rigid_motion;

TEST_CASE("the_composed_scenario_draws_its_panels_and_leaves_the_scene_as_it_found_it")
{
    stage headless;
    const std::size_t before = headless.descendants();

    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    REQUIRE(headless.descendants() > before);
    REQUIRE(fixture::composed_windows(composed) == std::vector<std::string>{"Two poses", "Decoupled path"});

    fixture::each_window_opens_one_panel(composed);

    composed->tear_down();

    CHECK(headless.descendants() == before);
}

TEST_CASE("both_paths_are_drawn_at_the_sampling_the_measurement_settled_and_meet_at_both_ends")
{
    stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    const std::vector<Eigen::Vector3d> coupled    = coupled_path(headless, composed);
    const std::vector<Eigen::Vector3d> decoupled  = decoupled_path(headless, composed);

    REQUIRE(two_pose_window::path_points == settled_path_points);
    REQUIRE(coupled.size() == settled_path_points);
    REQUIRE(decoupled.size() == settled_path_points);

    const transform start = panel_of(composed).start_pose();
    const transform end   = panel_of(composed).end_pose();

    CHECK(is_approx_equal((coupled.front() - start.block<3, 1>(0, 3)).norm(), 0.0, 1.0e-5));
    CHECK(is_approx_equal((coupled.back() - end.block<3, 1>(0, 3)).norm(), 0.0, 1.0e-5));
    CHECK(is_approx_equal((decoupled.front() - coupled.front()).norm(), 0.0, 1.0e-5));
    CHECK(is_approx_equal((decoupled.back() - coupled.back()).norm(), 0.0, 1.0e-5));
}

TEST_CASE("the_two_paths_part_between_the_ends_and_coincide_where_the_poses_differ_only_in_position")
{
    stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    CHECK_FALSE(same_line(coupled_path(headless, composed), decoupled_path(headless, composed)));
    CHECK(apart(coupled_path(headless, composed), decoupled_path(headless, composed)) > 0.1);

    // The two orientations typed alike, which leaves a displacement that is a pure slide: its screw
    // is the straight line between the two positions, and so is the decoupled path.
    for(std::size_t axis = 1; axis < 3; ++axis)
        fixture::type_into_slider_at(panel_of(composed), end_angle_controls + axis, "0");

    settled(composed);

    REQUIRE(panel_of(composed).state().end.euler_degrees.isZero());
    CHECK(same_line(coupled_path(headless, composed), decoupled_path(headless, composed)));
}

TEST_CASE("changing_a_pose_moves_both_paths_and_moving_the_parameter_moves_neither")
{
    stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    const std::vector<Eigen::Vector3d> coupled    = coupled_path(headless, composed);
    const std::vector<Eigen::Vector3d> decoupled  = decoupled_path(headless, composed);
    const transform resting                       = placed_in(composed).pose(two_pose_window::body_object);

    fixture::tweak_at(panel_of(composed), parameter_control, 4);
    settled(composed);

    REQUIRE(panel_of(composed).state().parameter != 0.f);
    CHECK_FALSE(is_approx_equal(placed_in(composed).pose(two_pose_window::body_object), resting));
    CHECK(same_line(coupled_path(headless, composed), coupled));
    CHECK(same_line(decoupled_path(headless, composed), decoupled));

    fixture::step_at(panel_of(composed), end_position_controls);
    settled(composed);

    REQUIRE(panel_of(composed).state().end.position.x() != -0.6f);
    CHECK_FALSE(same_line(coupled_path(headless, composed), coupled));
    CHECK_FALSE(same_line(decoupled_path(headless, composed), decoupled));
}

TEST_CASE("the_screw_path_moves_with_the_rigid_operations_and_leaves_the_decoupled_one_alone")
{
    capabilities rigid               = baseline();
    rigid.screw.matrix_logarithm_se3 = &reversed_screw;

    capabilities swept                   = baseline();
    swept.screw.matrix_exponential_screw = &turned_further;

    stage reference;
    stage displaced;
    stage driven;
    const std::shared_ptr<scene::preset> plain   = opened(reference, baseline());
    const std::shared_ptr<scene::preset> other   = opened(displaced, rigid);
    const std::shared_ptr<scene::preset> further = opened(driven, swept);

    CHECK_FALSE(same_line(coupled_path(displaced, other), coupled_path(reference, plain)));
    CHECK(same_line(decoupled_path(displaced, other), decoupled_path(reference, plain)));

    CHECK_FALSE(same_line(coupled_path(driven, further), coupled_path(reference, plain)));
    CHECK(same_line(decoupled_path(driven, further), decoupled_path(reference, plain)));
}

// The rotation logarithm is what the decoupled *motion* is made of and not what its *track* is: the
// drawn line runs through the sampled poses' translation columns, and under an interpolation that
// carries the position independently those are the straight line between the two positions whatever
// the orientation does. So this substitution moves neither drawn line, and the coupling it does have
// is on the poses handed to the route, which the window suite is where to read.
TEST_CASE("the_rotation_logarithm_is_what_the_decoupled_motion_is_made_of_and_not_what_its_track_is")
{
    capabilities turning               = baseline();
    turning.screw.matrix_logarithm_so3 = &reversed_turn;

    stage reference;
    stage rotated;
    const std::shared_ptr<scene::preset> plain  = opened(reference, baseline());
    const std::shared_ptr<scene::preset> turned = opened(rotated, turning);

    REQUIRE_FALSE(decoupled_path(reference, plain).empty());
    CHECK(same_line(coupled_path(rotated, turned), coupled_path(reference, plain)));
    CHECK(same_line(decoupled_path(rotated, turned), decoupled_path(reference, plain)));
}

TEST_CASE("two_equal_poses_draw_neither_path_and_leave_the_body_at_the_start_pose")
{
    stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    const transform start                         = panel_of(composed).start_pose();

    REQUIRE_FALSE(coupled_path(headless, composed).empty());

    fixture::tweak_at(panel_of(composed), parameter_control, 4);
    type_the_end_onto_the_start(composed);
    settled(composed);

    REQUIRE(panel_of(composed).state().parameter != 0.f);
    REQUIRE(is_approx_equal(panel_of(composed).end_pose(), start));
    CHECK(coupled_path(headless, composed).empty());
    CHECK(decoupled_path(headless, composed).empty());
    CHECK(is_approx_equal(placed_in(composed).pose(two_pose_window::body_object), start));
    CHECK_FALSE(panel_of(composed).reading().message.empty());
}

TEST_CASE("a_composition_that_reaches_for_no_second_window_has_no_decoupled_path_and_no_control_for_one")
{
    // The same objects and the same controls as the shipped scenario, with the second window simply
    // not composed. The fifth object is kept so that "nothing drew into it" is a reading rather than
    // an absence: the path is missing because nothing asked for it, not because there was nowhere to
    // put it.
    alone built;

    REQUIRE(fixture::line_points(built.target, built.body.name_of(two_pose_window::path_object)).size() == settled_path_points);
    CHECK(built.decoupled().empty());

    // Every control the panel offers, pressed in turn: none of them is one that turns a second path
    // on, because the panel that would carry one was never composed.
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);
    const fixture::drawing draw = [&built] { built.driving.render(); };

    CHECK(fixture::navigable_items(frames, draw) == parameter_control + 1u);

    fixture::drive_every_control(frames, draw);

    CHECK(built.decoupled().empty());
}
