#include "drawn_lines.h"
#include "composed_panels.h"

#include "praxis/presets/twist_axis.h"

#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/screw_window.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/twist_axis_window.h"

#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/labeled_value_window.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/core/Object3D.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::rigid_motion;

namespace {

// The line the scenario draws spans this much either way of the point on the axis it runs through.
constexpr double axis_half_length = 2.5;

// The object the curve is drawn under, which the scenario carries after the two the controls
// address by their own indices.
constexpr std::size_t path_object = 2;

// The angle stands below the two rows the twist's six components are typed into, so it is the third
// control a walk down the panel offers.
constexpr std::size_t angle_control = 2;

// The number of components a twist has, three to a row.
constexpr std::size_t twist_components = 6;
constexpr std::size_t row_components   = 3;

screw_axis mirrored_axis(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    return screw_axis_from_angular_linear(-w, v);
}

// Answers what nothing bound would answer, so every point sampled through it lands on its own seed
// and every pose driven through it stays where it started.
transform still(const screw_axis &, double)
{
    return transform::Identity();
}

// The angle control carries three decimals, so the angle it can be typed to stands a thousandth of
// a radian inside the limit it reaches; this is what that is worth in metres, with room over.
constexpr double typed_angle_slack = 1.0e-3;

frame_stencil &placed_in(const std::shared_ptr<scene::preset> &composed)
{
    return static_cast<frame_stencil &>(*composed->stencil);
}

twist_axis_window &panel_of(const std::shared_ptr<scene::preset> &composed)
{
    return static_cast<twist_axis_window &>(*composed->windows.front());
}

std::shared_ptr<scene::preset> opened(fixture::stage &headless, const capabilities &motions)
{
    const std::shared_ptr<scene::preset> composed = presets::twist_axis_preset(headless.site(), motions);

    REQUIRE(composed != nullptr);
    REQUIRE(composed->initialize().has_value());
    composed->windows.front()->initialize();

    return composed;
}

std::vector<Eigen::Vector3d> drawn_line(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed)
{
    return fixture::line_points(*headless.scene, placed_in(composed).name_of(twist_axis_window::axis_object));
}

std::vector<Eigen::Vector3d> drawn_curve(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed)
{
    return fixture::line_points(*headless.scene, placed_in(composed).name_of(path_object));
}

Eigen::Vector3d moved_to(const std::shared_ptr<scene::preset> &composed)
{
    return placed_in(composed).pose(twist_axis_window::body_object).block<3, 1>(0, 3);
}

// The angle typed into the control the panel draws it as, so what the object is driven to is what a
// person dragging that control would reach rather than a pose written past it.
void driven_to(const std::shared_ptr<scene::preset> &composed, const char *typed)
{
    fixture::type_into_slider_at(panel_of(composed), angle_control, typed);
}

std::vector<float> six_of(const std::shared_ptr<scene::preset> &composed)
{
    const scene::readout shown = panel_of(composed).reading();
    std::vector<float> values;

    REQUIRE(shown.message.empty());
    REQUIRE(shown.rows.size() == 1u);
    REQUIRE(shown.rows.front().size() == twist_components);
    for(const scene::labeled_value &cell : shown.rows.front())
        values.push_back(cell.value);

    return values;
}

// The axis the scenario reports, taken from the reading beside the drawing rather than derived here.
screw_axis derived(const std::shared_ptr<scene::preset> &composed)
{
    const std::vector<float> shown = six_of(composed);

    screw_axis named;
    for(Eigen::Index at = 0; at < named.size(); ++at)
        named[at] = shown[static_cast<std::size_t>(at)];

    return named;
}

}

TEST_CASE("the_composed_scenario_draws_its_panels_and_leaves_the_scene_as_it_found_it")
{
    fixture::stage headless;
    const std::size_t before = headless.descendants();

    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    REQUIRE(headless.descendants() > before);
    REQUIRE(fixture::composed_windows(composed) == std::vector<std::string>{"Twist", "Screw axis"});

    fixture::each_window_opens_one_panel(composed);

    composed->tear_down();

    CHECK(headless.descendants() == before);
}

TEST_CASE("the_drawn_line_runs_along_the_axis_the_twist_names_at_the_extent_the_scenario_draws")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    const std::vector<Eigen::Vector3d> ends       = drawn_line(headless, composed);
    const screw_axis named                        = derived(composed);
    const Eigen::Vector3d along                   = named.head<3>().normalized();

    REQUIRE(ends.size() == 2u);
    CHECK(is_approx_equal((ends.back() - ends.front()).norm(), 2.0 * axis_half_length, 1.0e-5));
    CHECK(is_approx_equal((ends.back() - ends.front()).normalized().dot(along), 1.0, 1.0e-6));

    // The point the line is drawn through lies on the axis, which the bound exponential is what
    // says: a point of a screw axis is carried along that axis and nowhere off it.
    const Eigen::Vector3d middle = 0.5 * (ends.front() + ends.back());
    const transform put          = matrix_exponential_screw(named, 0.7);
    const Eigen::Vector3d travel = put.topLeftCorner<3, 3>() * middle + put.block<3, 1>(0, 3) - middle;

    CHECK(travel.norm() > 1.0e-3);
    CHECK(is_approx_equal((travel - travel.dot(along) * along).norm(), 0.0, 1.0e-5));
}

TEST_CASE("the_drawn_line_follows_the_twist_and_is_left_alone_by_the_angle")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    const std::vector<Eigen::Vector3d> ends       = drawn_line(headless, composed);

    REQUIRE(ends.size() == 2u);

    fixture::tweak_at(panel_of(composed), angle_control, 4);

    REQUIRE(panel_of(composed).state().angle_radians != 0.f);
    CHECK(fixture::same_line(drawn_line(headless, composed), ends));

    fixture::type_component_at(panel_of(composed), 0, 0, "0.4");

    REQUIRE_FALSE(panel_of(composed).state().angular_part.x() == 0.f);
    CHECK_FALSE(fixture::same_line(drawn_line(headless, composed), ends));
}

TEST_CASE("the_angle_sweeps_the_moving_object_by_the_exponential_the_composition_bound")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    frame_stencil &placed                         = placed_in(composed);
    const transform start                         = placed.pose(twist_axis_window::body_object);

    fixture::tweak_at(panel_of(composed), angle_control, 4);

    const auto swept = static_cast<double>(panel_of(composed).state().angle_radians);

    REQUIRE(swept != 0.0);
    CHECK_FALSE(is_approx_equal(placed.pose(twist_axis_window::body_object), start));
    CHECK(is_approx_equal(placed.pose(twist_axis_window::body_object), matrix_exponential_screw(derived(composed), swept) * start, 1.0e-5));

    // The pose the scenario opens at is the pose the composition placed, which is where a zero
    // angle leaves the object.
    fixture::stage resting;

    CHECK(is_approx_equal(placed_in(opened(resting, baseline())).pose(twist_axis_window::body_object), start));
}

TEST_CASE("a_substituted_axis_binding_changes_the_line_drawn_and_the_reading_beside_it_together")
{
    capabilities substituted                         = baseline();
    substituted.screw.screw_axis_from_angular_linear = &mirrored_axis;

    fixture::stage reference;
    fixture::stage perturbed;
    const std::shared_ptr<scene::preset> plain = opened(reference, baseline());
    const std::shared_ptr<scene::preset> other = opened(perturbed, substituted);

    const std::vector<Eigen::Vector3d> ends  = drawn_line(reference, plain);
    const std::vector<Eigen::Vector3d> moved = drawn_line(perturbed, other);

    REQUIRE(ends.size() == 2u);
    REQUIRE(moved.size() == 2u);
    CHECK_FALSE(fixture::same_line(moved, ends));
    CHECK(six_of(other) != six_of(plain));
    CHECK(fixture::geometry_of(*other->windows.back()) != fixture::geometry_of(*plain->windows.back()));
}

TEST_CASE("a_twist_typed_to_zero_leaves_no_line_drawn_and_the_moving_object_where_it_was")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    // Five of the six, so the last twist naming an axis stands one component away from naming none.
    // A row is entered at its leftmost and the components beside it are stepped along it.
    for(std::size_t component = 0; component + 1u < twist_components; ++component)
        fixture::type_component_at(panel_of(composed), component / row_components, component % row_components, "0");

    const transform swept = placed_in(composed).pose(twist_axis_window::body_object);

    REQUIRE(drawn_line(headless, composed).size() == 2u);

    fixture::type_component_at(panel_of(composed), twist_components / row_components - 1u, row_components - 1u, "0");

    CHECK(panel_of(composed).state().angular_part.isZero());
    CHECK(panel_of(composed).state().linear_part.isZero());
    CHECK(drawn_line(headless, composed).empty());
    CHECK(is_approx_equal(placed_in(composed).pose(twist_axis_window::body_object), swept));
    CHECK_FALSE(panel_of(composed).reading().message.empty());
}

TEST_CASE("an_unbound_axis_operation_leaves_the_scenario_drawing_nothing_rather_than_a_line")
{
    capabilities unbound                         = baseline();
    unbound.screw.screw_axis_from_angular_linear = &inert::screw_axis_from_angular_linear;

    fixture::stage headless;
    fixture::stage reference;
    const std::shared_ptr<scene::preset> composed = opened(headless, unbound);
    const transform start                         = placed_in(composed).pose(twist_axis_window::body_object);

    REQUIRE_FALSE(drawn_line(reference, opened(reference, baseline())).empty());
    CHECK(drawn_line(headless, composed).empty());

    fixture::tweak_at(panel_of(composed), angle_control, 4);

    REQUIRE(panel_of(composed).state().angle_radians != 0.f);
    CHECK(is_approx_equal(placed_in(composed).pose(twist_axis_window::body_object), start));
}

TEST_CASE("the_drawn_curve_carries_the_moving_object_at_every_angle_the_control_reaches")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    const std::vector<Eigen::Vector3d> curve      = drawn_curve(headless, composed);
    const std::vector<Eigen::Vector3d> ends       = drawn_line(headless, composed);

    REQUIRE(curve.size() > 2u);
    REQUIRE(ends.size() == 2u);

    const double resolution = fixture::sampling_resolution(curve, ends);

    REQUIRE(resolution > 0.0);
    CHECK(fixture::from_curve(curve, moved_to(composed)) <= resolution);

    for(const char *angle : {"1.000", "-2.500", "3.000", "-5.750"})
    {
        INFO(angle);
        driven_to(composed, angle);

        REQUIRE(panel_of(composed).state().angle_radians != 0.f);
        CHECK(fixture::from_curve(curve, moved_to(composed)) <= resolution);
        CHECK(fixture::same_line(drawn_curve(headless, composed), curve));
    }
}

TEST_CASE("the_drawn_curve_reaches_as_far_as_the_angle_control_does_at_either_end")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    const std::vector<Eigen::Vector3d> curve      = drawn_curve(headless, composed);

    REQUIRE(curve.size() > 2u);

    driven_to(composed, "6.283");

    CHECK(is_approx_equal(static_cast<double>(panel_of(composed).state().angle_radians), screw_window::angle_limit_radians, 1.0e-3));
    CHECK((moved_to(composed) - curve.back()).norm() <= typed_angle_slack);

    driven_to(composed, "-6.283");

    CHECK(is_approx_equal(static_cast<double>(panel_of(composed).state().angle_radians), -screw_window::angle_limit_radians, 1.0e-3));
    CHECK((moved_to(composed) - curve.front()).norm() <= typed_angle_slack);
}

TEST_CASE("the_curve_and_the_line_naming_the_axis_are_rebuilt_together_and_the_angle_leaves_both_alone")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    const std::vector<Eigen::Vector3d> ends       = drawn_line(headless, composed);
    const std::vector<Eigen::Vector3d> curve      = drawn_curve(headless, composed);

    REQUIRE(ends.size() == 2u);
    REQUIRE(curve.size() > 2u);

    driven_to(composed, "3.000");

    REQUIRE(panel_of(composed).state().angle_radians != 0.f);
    CHECK(fixture::same_line(drawn_line(headless, composed), ends));
    CHECK(fixture::same_line(drawn_curve(headless, composed), curve));

    fixture::type_component_at(panel_of(composed), 0, 0, "0.4");

    REQUIRE_FALSE(panel_of(composed).state().angular_part.x() == 0.f);
    CHECK_FALSE(fixture::same_line(drawn_line(headless, composed), ends));
    CHECK_FALSE(fixture::same_line(drawn_curve(headless, composed), curve));
}

TEST_CASE("a_twist_typed_to_zero_leaves_neither_the_curve_nor_the_line_naming_the_axis_drawn")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    for(std::size_t component = 0; component + 1u < twist_components; ++component)
        fixture::type_component_at(panel_of(composed), component / row_components, component % row_components, "0");

    REQUIRE(drawn_line(headless, composed).size() == 2u);
    REQUIRE(drawn_curve(headless, composed).size() > 2u);

    fixture::type_component_at(panel_of(composed), twist_components / row_components - 1u, row_components - 1u, "0");

    CHECK(drawn_line(headless, composed).empty());
    CHECK(drawn_curve(headless, composed).empty());
}

// The one substitution that catches a curve drawn from a formula rather than from the term the
// moving object is driven by: an object that does not move and a curve that still spirals would be
// two answers to one question.
TEST_CASE("the_curve_and_the_moving_object_both_follow_the_exponential_the_composition_bound")
{
    capabilities motionless                   = baseline();
    motionless.screw.matrix_exponential_screw = &still;

    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, motionless);
    const std::vector<Eigen::Vector3d> curve      = drawn_curve(headless, composed);
    const Eigen::Vector3d start                   = moved_to(composed);

    REQUIRE(curve.size() > 2u);
    for(const Eigen::Vector3d &at : curve)
        CHECK(is_approx_equal((at - start).norm(), 0.0, 1.0e-6));

    driven_to(composed, "3.000");

    REQUIRE(panel_of(composed).state().angle_radians != 0.f);
    CHECK(is_approx_equal((moved_to(composed) - start).norm(), 0.0, 1.0e-6));

    fixture::stage moving;
    const std::vector<Eigen::Vector3d> drawn = drawn_curve(moving, opened(moving, baseline()));

    REQUIRE(drawn.size() == curve.size());
    CHECK(fixture::coarsest_step(drawn) > 1.0e-3);
}
