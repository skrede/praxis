#include "drawn_lines.h"
#include "composed_panels.h"

#include "praxis/presets/screw.h"

#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/screw_window.h"
#include "praxis/rigid_motion/frame_stencil.h"

#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/scene/preset.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/core/Object3D.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <numbers>
#include <optional>
#include <string_view>

using namespace praxis;
using namespace praxis::rigid_motion;

namespace {

// What the composition handed the axis slot, so the case knows the pitch the scenario asked for
// rather than repeating a constant the scenario owns.
double asked_pitch = 0.0;

expected<screw_axis, refusal> recording_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h)
{
    asked_pitch = h;

    return screw_axis_from_point_direction_pitch(q, s, h);
}

expected<screw_axis, refusal> flattened_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double)
{
    return screw_axis_from_point_direction_pitch(q, s, 0.0);
}

// Answers what nothing bound would answer, so every point sampled through it lands on its own seed
// and every pose driven through it stays where it started.
transform still(const screw_axis &, double)
{
    return transform::Identity();
}

transform turned_further(const screw_axis &s, double theta)
{
    return matrix_exponential_screw(s, theta + 1.0);
}

// The items the panel offers a walk down it, which the walk itself is what says: the point's row,
// the direction's row, the pitch and the angle. The button beside the direction stands on that row's
// own line, which a walk down the panel does not reach.
constexpr std::size_t point_control     = 0;
constexpr std::size_t direction_control = 1;
constexpr std::size_t pitch_control     = 2;
constexpr std::size_t angle_control     = 3;
constexpr std::size_t panel_items       = 4;

// The number of components a point or a direction is typed into, three to a row.
constexpr std::size_t row_components = 3;

// The object carrying the located axis point, which the scenario composes after the three the
// controls address by their own indices.
constexpr std::size_t located_object = 3;

frame_stencil &placed_in(const std::shared_ptr<scene::preset> &composed)
{
    return static_cast<frame_stencil &>(*composed->stencil);
}

screw_window &panel_of(const std::shared_ptr<scene::preset> &composed)
{
    return static_cast<screw_window &>(*composed->windows.front());
}

double axis_distance(const Eigen::Vector3d &p)
{
    return std::hypot(p.x(), p.y());
}

std::shared_ptr<scene::preset> opened(fixture::stage &headless, const capabilities &motions)
{
    const std::shared_ptr<scene::preset> composed = presets::screw_preset(headless.site(), motions);

    REQUIRE(composed != nullptr);
    REQUIRE(composed->initialize().has_value());
    composed->windows.front()->initialize();

    return composed;
}

// Read in the frame the geometry was built in, which is the frame the stencil's poses are written in:
// the object carrying the curve is never placed.
std::vector<Eigen::Vector3d> drawn_curve(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed)
{
    return fixture::line_points(*headless.scene, placed_in(composed).name_of(screw_window::thread_object));
}

// The line naming the axis is carried by a posed object, so its ends are read after the placement
// has been pushed into the scene rather than out of the buffer it was built in.
std::vector<Eigen::Vector3d> drawn_axis(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed)
{
    placed_in(composed).render();
    headless.scene->updateMatrixWorld(true);

    return fixture::line_in_world(*headless.scene, placed_in(composed).name_of(screw_window::axis_object));
}

Eigen::Vector3d moved_to(const std::shared_ptr<scene::preset> &composed)
{
    return placed_in(composed).pose(screw_window::body_object).block<3, 1>(0, 3);
}

// The connector out of the origin and the mark on its far end, both built in the frame the stencil's
// poses are written in under an object that is never placed. Reading the connector as a line is what
// says it was drawn as one: a solid would not answer here at all.
std::vector<Eigen::Vector3d> drawn_connector(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed)
{
    return fixture::line_points(*headless.scene, placed_in(composed).name_of(located_object));
}

std::optional<Eigen::Vector3d> drawn_mark(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed)
{
    return fixture::mesh_position(*headless.scene, placed_in(composed).name_of(located_object), "point");
}

// A row is entered at its leftmost and the components beside it are stepped along it.
void typed_row(const std::shared_ptr<scene::preset> &composed, std::size_t control, const char *first, const char *second, const char *third)
{
    fixture::type_component_at(panel_of(composed), control, 0, first);
    fixture::type_component_at(panel_of(composed), control, 1, second);
    fixture::type_component_at(panel_of(composed), control, 2, third);
}

// The angle typed into the control the panel draws it as, so what the body is driven to is what a
// person dragging that control would reach rather than a pose written past it.
void driven_to(const std::shared_ptr<scene::preset> &composed, const char *typed)
{
    fixture::type_into_slider_at(panel_of(composed), angle_control, typed);
}

// The drawn curve describes the motion when the body stands on it, within the sagitta of one sampled
// step, which the drawn geometry itself states.
void stands_on_the_drawn_curve(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed)
{
    const std::vector<Eigen::Vector3d> curve = drawn_curve(headless, composed);
    const std::vector<Eigen::Vector3d> ends  = drawn_axis(headless, composed);

    REQUIRE(curve.size() > 2u);
    REQUIRE(ends.size() == 2u);

    const double resolution = fixture::sampling_resolution(curve, ends);

    REQUIRE(resolution > 0.0);
    CHECK(fixture::from_curve(curve, moved_to(composed)) <= resolution);
}

// Exactly representable in single precision, so what the controls carry as floats reaches the
// operations as the doubles this file recomputes with.
const screw_window::settings driven{Eigen::Vector3f{0.125f, 0.f, 0.5f}, Eigen::Vector3f{0.f, 0.f, 1.f}, 0.25f, 0.75f};

const Eigen::Vector3d driven_point{0.125, 0.0, 0.5};

// The line the axis object carries spans this much either way of the point it passes through.
constexpr double axis_half_length = 2.5;

double apart(const Eigen::Vector3d &first, const Eigen::Vector3d &second)
{
    return (first - second).norm();
}

// Every line under the named object rather than the first one, which is what says how many curves
// are drawn: reading the first answers the same points whether there is one of them or three.
std::size_t lines_under(threepp::Scene &target, std::string_view named)
{
    std::size_t drawn       = 0;
    threepp::Object3D *node = target.getObjectByName<threepp::Object3D>(std::string(named));
    if(node == nullptr)
        return drawn;

    node->traverse(
            [&drawn](threepp::Object3D &at)
            {
                if(at.type() == "Line")
                    ++drawn;
            });

    return drawn;
}

}

TEST_CASE("the_driven_pose_is_the_exponential_of_the_named_axis_applied_to_the_start_pose")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = presets::screw_preset(headless.site(), baseline());

    REQUIRE(composed != nullptr);
    REQUIRE(composed->initialize().has_value());

    frame_stencil &placed = placed_in(composed);
    const transform start = placed.pose(screw_window::body_object);
    screw_window driving("Driven", placed, baseline(), screw_window::axis_route{}, driven);

    driving.initialize();

    const expected<screw_axis, refusal> about = screw_axis_from_point_direction_pitch(driven_point, Eigen::Vector3d::UnitZ(), 0.25);

    REQUIRE(about.has_value());
    CHECK(is_approx_equal(placed.pose(screw_window::body_object), matrix_exponential_screw(*about, 0.75) * start));
}

TEST_CASE("the_drive_runs_through_the_exponential_the_composition_bound")
{
    fixture::stage headless;
    capabilities motions                          = baseline();
    motions.screw.matrix_exponential_screw        = &turned_further;
    const std::shared_ptr<scene::preset> composed = presets::screw_preset(headless.site(), motions);

    REQUIRE(composed != nullptr);
    REQUIRE(composed->initialize().has_value());

    frame_stencil &placed = placed_in(composed);
    const transform start = placed.pose(screw_window::body_object);
    screw_window driving("Driven", placed, motions, screw_window::axis_route{}, driven);

    driving.initialize();

    const expected<screw_axis, refusal> about = screw_axis_from_point_direction_pitch(driven_point, Eigen::Vector3d::UnitZ(), 0.25);

    REQUIRE(about.has_value());
    CHECK(is_approx_equal(placed.pose(screw_window::body_object), matrix_exponential_screw(*about, 1.75) * start));
}

TEST_CASE("an_axis_the_operations_refuse_to_name_leaves_the_composed_poses_where_they_were")
{
    fixture::stage headless;
    capabilities motions                                = baseline();
    motions.screw.screw_axis_from_point_direction_pitch = &inert::screw_axis_from_point_direction_pitch;
    const std::shared_ptr<scene::preset> composed       = presets::screw_preset(headless.site(), motions);

    REQUIRE(composed != nullptr);
    REQUIRE(composed->initialize().has_value());

    frame_stencil &placed = placed_in(composed);
    const transform start = placed.pose(screw_window::body_object);
    screw_window driving("Driven", placed, motions, screw_window::axis_route{}, driven);

    driving.initialize();

    CHECK(is_approx_equal(placed.pose(screw_window::body_object), start));
    CHECK(is_approx_equal(placed.pose(screw_window::axis_object), transform::Identity()));
}

TEST_CASE("a_direction_of_zero_length_names_no_axis_and_leaves_the_composed_poses_where_they_were")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = presets::screw_preset(headless.site(), baseline());

    REQUIRE(composed != nullptr);
    REQUIRE(composed->initialize().has_value());

    frame_stencil &placed = placed_in(composed);
    const transform start = placed.pose(screw_window::body_object);
    screw_window driving("Driven", placed, baseline(), screw_window::axis_route{}, screw_window::settings{driven.point, Eigen::Vector3f::Zero(), driven.pitch, driven.angle_radians});

    driving.initialize();

    CHECK(is_approx_equal(placed.pose(screw_window::body_object), start));
    CHECK(is_approx_equal(placed.pose(screw_window::axis_object), transform::Identity()));
}

TEST_CASE("a_thread_of_zero_pitch_stands_at_one_distance_from_the_axis_and_closes_on_itself")
{
    fixture::stage headless;
    capabilities motions                                = baseline();
    motions.screw.screw_axis_from_point_direction_pitch = &flattened_axis;

    const std::shared_ptr<scene::preset> composed = opened(headless, motions);
    const std::vector<Eigen::Vector3d> points     = drawn_curve(headless, composed);
    const double resting                          = moved_to(composed).z();

    REQUIRE(points.size() > 2u);
    CHECK(axis_distance(points.front()) > 0.0);

    for(const Eigen::Vector3d &at : points)
    {
        CHECK(is_approx_equal(axis_distance(at), axis_distance(points.front()), 1.0e-6));
        CHECK(is_approx_equal(at.z(), resting, 1.0e-6));
    }

    CHECK(is_approx_equal(axis_distance(points.back() - points.front()), 0.0, 1.0e-6));
}

TEST_CASE("a_thread_advances_along_the_axis_by_the_pitch_over_a_turn_and_holds_its_distance_from_it")
{
    fixture::stage headless;
    capabilities motions                                = baseline();
    motions.screw.screw_axis_from_point_direction_pitch = &recording_axis;
    asked_pitch                                         = 0.0;

    const std::vector<Eigen::Vector3d> points = drawn_curve(headless, opened(headless, motions));
    const std::size_t over_a_turn             = (points.size() - 1u) / 2u;

    REQUIRE(asked_pitch > 0.0);
    REQUIRE(over_a_turn > 0u);

    for(std::size_t at = 0; at + over_a_turn < points.size(); ++at)
    {
        CHECK(is_approx_equal(points[at + over_a_turn].z() - points[at].z(), 2.0 * std::numbers::pi * asked_pitch, 1.0e-6));
        CHECK(is_approx_equal(axis_distance(points[at]), axis_distance(points.front()), 1.0e-6));
    }
}

TEST_CASE("the_thread_object_carries_one_drawn_curve")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    CHECK(lines_under(*headless.scene, placed_in(composed).name_of(screw_window::thread_object)) == 1u);
}

TEST_CASE("the_panel_offers_the_point_the_direction_the_pitch_and_the_angle_to_a_walk")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    {
        tests::imgui_frame frames;
        frames.assert_on_frame_faults(true);

        CHECK(fixture::navigable_items(frames, [&composed] { panel_of(composed).render(); }) == panel_items);
    }

    typed_row(composed, point_control, "0.375", "0", "0.5");

    CHECK(panel_of(composed).state().point.x() == 0.375f);

    typed_row(composed, direction_control, "0", "0.25", "1");

    CHECK(panel_of(composed).state().direction.y() == 0.25f);

    fixture::type_component_at(panel_of(composed), pitch_control, 0, "0.2");

    CHECK(panel_of(composed).state().pitch == 0.2f);

    driven_to(composed, "1.000");

    CHECK(is_approx_equal(static_cast<double>(panel_of(composed).state().angle_radians), 1.0, 1.0e-3));
}

TEST_CASE("a_moved_axis_point_leaves_the_body_on_the_curve_at_every_angle_the_control_reaches")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    stands_on_the_drawn_curve(headless, composed);

    typed_row(composed, point_control, "0", "0.5", "0");

    REQUIRE(panel_of(composed).state().point.y() == 0.5f);

    stands_on_the_drawn_curve(headless, composed);

    for(const char *angle : {"1.000", "-2.500", "3.000", "-5.750"})
    {
        INFO(angle);
        driven_to(composed, angle);

        REQUIRE(panel_of(composed).state().angle_radians != 0.f);

        stands_on_the_drawn_curve(headless, composed);
    }
}

TEST_CASE("a_moved_axis_direction_leaves_the_body_on_the_curve_at_every_angle_the_control_reaches")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    typed_row(composed, direction_control, "0", "1", "1");

    REQUIRE(panel_of(composed).state().direction.y() == 1.f);

    stands_on_the_drawn_curve(headless, composed);

    for(const char *angle : {"1.000", "-2.500", "3.000"})
    {
        INFO(angle);
        driven_to(composed, angle);

        REQUIRE(panel_of(composed).state().angle_radians != 0.f);

        stands_on_the_drawn_curve(headless, composed);
    }
}

TEST_CASE("a_moved_pitch_beside_a_moved_point_leaves_the_body_on_the_curve")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    typed_row(composed, point_control, "0", "0.5", "0");
    fixture::type_component_at(panel_of(composed), pitch_control, 0, "0.25");

    REQUIRE(panel_of(composed).state().point.y() == 0.5f);
    REQUIRE(panel_of(composed).state().pitch == 0.25f);

    stands_on_the_drawn_curve(headless, composed);

    for(const char *angle : {"1.000", "-2.500"})
    {
        INFO(angle);
        driven_to(composed, angle);

        REQUIRE(panel_of(composed).state().angle_radians != 0.f);

        stands_on_the_drawn_curve(headless, composed);
    }
}

TEST_CASE("the_angle_leaves_the_drawn_curve_and_the_drawn_axis_line_exactly_as_they_were")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    typed_row(composed, point_control, "0", "0.5", "0");

    const std::vector<Eigen::Vector3d> curve = drawn_curve(headless, composed);
    const std::vector<Eigen::Vector3d> ends  = drawn_axis(headless, composed);

    REQUIRE(curve.size() > 2u);
    REQUIRE(ends.size() == 2u);

    driven_to(composed, "3.000");

    REQUIRE(panel_of(composed).state().angle_radians != 0.f);
    CHECK(fixture::same_line(drawn_curve(headless, composed), curve));
    CHECK(fixture::same_line(drawn_axis(headless, composed), ends));

    typed_row(composed, point_control, "0", "0.75", "0");

    REQUIRE(panel_of(composed).state().point.y() == 0.75f);
    CHECK_FALSE(fixture::same_line(drawn_curve(headless, composed), curve));
    CHECK_FALSE(fixture::same_line(drawn_axis(headless, composed), ends));
}

// The one substitution that catches a thread drawn from a formula: a body that does not move and
// threads that still spiral would be two answers to one question.
TEST_CASE("the_threads_and_the_body_both_follow_the_exponential_the_composition_bound")
{
    fixture::stage headless;
    capabilities motions                          = baseline();
    motions.screw.matrix_exponential_screw        = &still;
    const std::shared_ptr<scene::preset> composed = opened(headless, motions);
    frame_stencil &placed                         = placed_in(composed);
    const transform start                         = placed.pose(screw_window::body_object);

    screw_window driving("Driven", placed, motions, screw_window::axis_route{}, driven);

    driving.initialize();

    const std::vector<Eigen::Vector3d> sampled = drawn_curve(headless, composed);

    REQUIRE(sampled.size() > 2u);
    CHECK(is_approx_equal(placed.pose(screw_window::body_object), start));
    for(const Eigen::Vector3d &at : sampled)
        CHECK(is_approx_equal(axis_distance(at - sampled.front()) + std::abs(at.z() - sampled.front().z()), 0.0, 1.0e-6));

    fixture::stage moving;
    const std::vector<Eigen::Vector3d> drawn = drawn_curve(moving, opened(moving, baseline()));

    REQUIRE(drawn.size() == sampled.size());
    CHECK(std::abs(drawn.back().z() - drawn.front().z()) > 1.0e-3);
}

TEST_CASE("the_axis_carries_a_drawn_line_along_itself_that_a_thread_rebuild_leaves_alone")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    frame_stencil &placed                         = placed_in(composed);

    const std::vector<Eigen::Vector3d> ends   = fixture::line_points(*headless.scene, placed.name_of(screw_window::axis_object));
    const std::vector<Eigen::Vector3d> spiral = drawn_curve(headless, composed);

    REQUIRE(ends.size() == 2u);
    REQUIRE(spiral.size() > 2u);
    CHECK(is_approx_equal(axis_distance(ends.front()), 0.0, 1.0e-6));
    CHECK(is_approx_equal(axis_distance(ends.back()), 0.0, 1.0e-6));
    CHECK(is_approx_equal(apart(ends.back(), ends.front()), 2.0 * axis_half_length, 1.0e-6));

    screw_window turning("Driven", placed, baseline(), screw_window::axis_route{}, driven);
    turning.initialize();
    composed->windows.front()->initialize();
    const std::vector<Eigen::Vector3d> after = fixture::line_points(*headless.scene, placed.name_of(screw_window::axis_object));

    REQUIRE(after.size() == 2u);
    CHECK(is_approx_equal(apart(after.front(), ends.front()), 0.0, 1.0e-6));
    CHECK(is_approx_equal(apart(after.back(), ends.back()), 0.0, 1.0e-6));
    CHECK(drawn_curve(headless, composed).size() == spiral.size());
}

TEST_CASE("a_frame_stands_at_the_scene_origin_and_the_axis_object_carries_no_triad")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    frame_stencil &placed                         = placed_in(composed);

    REQUIRE(placed.fixed_frame_name() == std::string_view("Space"));
    CHECK(headless.scene->getObjectByName("Space") != nullptr);
    CHECK_FALSE(placed.axes_shown(screw_window::axis_object));
    CHECK(placed.axes_shown(screw_window::body_object));
}

TEST_CASE("the_connector_spans_the_origin_and_the_axis_point_with_the_mark_standing_on_it")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    const std::vector<Eigen::Vector3d> spans      = drawn_connector(headless, composed);
    const std::optional<Eigen::Vector3d> mark     = drawn_mark(headless, composed);
    const Eigen::Vector3d at                      = panel_of(composed).state().point.cast<double>();

    REQUIRE(spans.size() == 2u);
    REQUIRE(mark.has_value());
    CHECK(is_approx_equal(apart(spans.front(), Eigen::Vector3d::Zero()), 0.0, 1.0e-6));
    CHECK(is_approx_equal(apart(spans.back(), at), 0.0, 1.0e-6));
    CHECK(is_approx_equal(apart(*mark, at), 0.0, 1.0e-6));
}

TEST_CASE("the_connector_and_the_mark_follow_the_axis_point_when_it_is_moved_through_the_panel")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    const Eigen::Vector3d moved{-0.25, 0.5, 0.125};

    typed_row(composed, point_control, "-0.25", "0.5", "0.125");

    REQUIRE(panel_of(composed).state().point.cast<double>().isApprox(moved));

    const std::vector<Eigen::Vector3d> spans  = drawn_connector(headless, composed);
    const std::optional<Eigen::Vector3d> mark = drawn_mark(headless, composed);

    REQUIRE(spans.size() == 2u);
    REQUIRE(mark.has_value());
    CHECK(is_approx_equal(apart(spans.front(), Eigen::Vector3d::Zero()), 0.0, 1.0e-6));
    CHECK(is_approx_equal(apart(spans.back(), moved), 0.0, 1.0e-6));
    CHECK(is_approx_equal(apart(*mark, moved), 0.0, 1.0e-6));
}

TEST_CASE("neither_the_connector_nor_the_mark_is_drawn_where_the_controls_name_no_axis")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    REQUIRE(drawn_connector(headless, composed).size() == 2u);
    REQUIRE(drawn_mark(headless, composed).has_value());

    typed_row(composed, direction_control, "0", "0", "0");

    REQUIRE(panel_of(composed).state().direction.isZero());
    CHECK(drawn_connector(headless, composed).empty());
    CHECK_FALSE(drawn_mark(headless, composed).has_value());
    CHECK(drawn_curve(headless, composed).empty());
}

TEST_CASE("the_drawn_axis_line_is_carried_onto_the_named_point_and_direction_by_the_object_holding_it")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    frame_stencil &placed                         = placed_in(composed);

    const screw_window::settings tilted{Eigen::Vector3f{0.25f, -0.5f, 0.75f}, Eigen::Vector3f{0.f, 1.f, 1.f}, 0.25f, 0.75f};
    screw_window driving("Driven", placed, baseline(), screw_window::axis_route{}, tilted);

    driving.initialize();

    const std::vector<Eigen::Vector3d> ends = drawn_axis(headless, composed);
    const Eigen::Vector3d along             = tilted.direction.cast<double>().normalized();
    const Eigen::Vector3d through           = tilted.point.cast<double>();

    REQUIRE(ends.size() == 2u);
    CHECK(is_approx_equal(apart(ends.front(), through - axis_half_length * along), 0.0, 1.0e-5));
    CHECK(is_approx_equal(apart(ends.back(), through + axis_half_length * along), 0.0, 1.0e-5));
}
