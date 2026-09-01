#include "drawn_lines.h"
#include "labeled_panels.h"
#include "composed_panels.h"

#include "praxis/presets/rotation_axis.h"

#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/rotation_axis_window.h"

#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/core/Object3D.hpp>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>

using namespace praxis;
using namespace praxis::rigid_motion;

namespace {

// The direction is the row a walk down the panel reaches first, the angle stands below it, and the
// three switches stand below that.
constexpr std::size_t direction_control = 0;
constexpr std::size_t angle_control     = 1;

// The two objects the scenario carries its own arrows on, standing after the frame the controls
// drive, and the three carrying the arcs those arrows trace.
constexpr std::size_t axis_object       = 1;
constexpr std::size_t coordinate_object = 2;
constexpr std::size_t arc_x_object      = 3;
constexpr std::size_t arc_y_object      = 4;
constexpr std::size_t arc_z_object      = 5;

// The end of one of the frame's own arrows, in the frame's own coordinates, and the object carrying
// the arc that end traces.
struct traced
{
    std::size_t object;
    Eigen::Vector3d carried;
};

const std::vector<traced> arcs{{arc_x_object, Eigen::Vector3d{axes_settings{}.axis_length, 0.0, 0.0}},
                               {arc_y_object, Eigen::Vector3d{0.0, axes_settings{}.axis_length, 0.0}},
                               {arc_z_object, Eigen::Vector3d{0.0, 0.0, axes_settings{}.axis_length}}};

frame_stencil &placed_in(const std::shared_ptr<scene::preset> &composed)
{
    return static_cast<frame_stencil &>(*composed->stencil);
}

rotation_axis_window &panel_of(const std::shared_ptr<scene::preset> &composed)
{
    return static_cast<rotation_axis_window &>(*composed->windows.front());
}

std::shared_ptr<scene::preset> opened(fixture::stage &headless, const capabilities &motions)
{
    const std::shared_ptr<scene::preset> composed = presets::rotation_axis_preset(headless.site(), motions);

    REQUIRE(composed != nullptr);
    REQUIRE(composed->initialize().has_value());
    composed->windows.front()->initialize();

    return composed;
}

const transform &standing(const std::shared_ptr<scene::preset> &composed)
{
    return placed_in(composed).pose(rotation_axis_window::frame_object);
}

// The angle typed into the control the panel draws it as, so what the frame is turned to is what a
// person dragging that control would reach rather than a pose written past it.
void driven_to(const std::shared_ptr<scene::preset> &composed, const char *typed)
{
    fixture::type_into_slider_at(panel_of(composed), angle_control, typed);
}

// The control returning the angle to zero stands beside the angle rather than under it, so a walk
// down the panel never reaches it: it is stepped onto from the row it acts on, and the identifier it
// is then standing on is what says it is that control and not another.
void press_reset(const std::shared_ptr<scene::preset> &composed)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const fixture::drawing over = [&composed] { panel_of(composed).render(); };

    fixture::stand_below_top(frames, over, angle_control);
    fixture::tap(frames, over, ImGuiKey_RightArrow);

    REQUIRE(fixture::standing_on() == fixture::control_id(panel_of(composed).display_name().c_str(), "Reset"));

    fixture::tap(frames, over, ImGuiKey_Space);
}

// A row is entered at its leftmost and the components beside it are stepped along it.
void typed_direction(const std::shared_ptr<scene::preset> &composed, const char *first, const char *second, const char *third)
{
    fixture::type_component_at(panel_of(composed), direction_control, 0, first);
    fixture::type_component_at(panel_of(composed), direction_control, 1, second);
    fixture::type_component_at(panel_of(composed), direction_control, 2, third);
}

// The axis the composed controls name, taken as a unit vector because that is what the exponential
// coordinate is measured along.
Eigen::Vector3d about(const std::shared_ptr<scene::preset> &composed)
{
    return panel_of(composed).state().direction.cast<double>().normalized();
}

transform turned(const Eigen::Vector3d &axis, double theta_radians, const transform &start)
{
    return transformation_matrix_from_rotation(matrix_exponential_so3(axis, theta_radians)) * start;
}

// Answers the turn a radian past the one asked for, so what reads this slot moves and what is spelled
// any other way stays where it was.
rotation turned_further(const Eigen::Vector3d &axis, double theta_radians)
{
    return matrix_exponential_so3(axis, theta_radians + 1.0);
}

// A part of an arrow stands under a posed object, so it is read after the placement has been pushed
// into the scene rather than out of the geometry it was built in.
std::optional<Eigen::Vector3d> arrow_part(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed, std::size_t index, const char *part)
{
    placed_in(composed).render();
    headless.scene->updateMatrixWorld(true);

    return fixture::mesh_in_world(*headless.scene, placed_in(composed).name_of(index), part);
}

std::optional<Eigen::Vector3d> arrow_head(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed, std::size_t index)
{
    return arrow_part(headless, composed, index, "head");
}

// How far the arrow reaches, tip included. The stem stands from the origin to its own top and the
// head stands centred on what is left, so twice the gap between where the two parts stand is the
// whole drawn arrow.
double reach_of(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed, std::size_t index)
{
    const std::optional<Eigen::Vector3d> stem = arrow_part(headless, composed, index, "stem");
    const std::optional<Eigen::Vector3d> head = arrow_part(headless, composed, index, "head");

    REQUIRE(stem.has_value());
    REQUIRE(head.has_value());

    return 2.0 * (head->norm() - stem->norm());
}

Eigen::Vector3d along_of(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed, std::size_t index)
{
    const std::optional<Eigen::Vector3d> at = arrow_head(headless, composed, index);

    REQUIRE(at.has_value());
    REQUIRE(at->norm() > 0.0);

    return at->normalized();
}

// A drawn arrow read back out of its own two meshes: the stem stands from the origin to its own top
// and the head is a cone standing on what is left of it, both built along the renderer's +Y, and the
// whole carried onto the direction the arrow was placed along.
struct drawn_profile
{
    bool material;
    Eigen::Vector3d along;
    double stem_radius;
    double stem_top;
    double head_bottom;
    double head_top;
    double head_radius;
};

drawn_profile profile_of(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed, std::size_t index)
{
    const std::optional<Eigen::Vector3d> at = arrow_part(headless, composed, index, "head");
    threepp::Object3D *stem                 = fixture::first_mesh_under(*headless.scene, placed_in(composed).name_of(index), "stem");
    threepp::Object3D *head                 = fixture::first_mesh_under(*headless.scene, placed_in(composed).name_of(index), "head");

    if(stem == nullptr || head == nullptr || !fixture::drawn(head) || !at || at->norm() <= 0.0)
        return drawn_profile{false, Eigen::Vector3d::Zero(), 0.0, 0.0, 0.0, 0.0, 0.0};

    // An arrow shorter than its own head is drawn as the head alone, and its stem is no material at
    // all rather than a disc of the stem's radius at the origin.
    const double stem_height = 2.0 * fixture::mesh_half_extent(stem).y() * static_cast<double>(stem->scale.y);
    const double stem_radius = fixture::mesh_half_extent(stem).x() * static_cast<double>(stem->scale.x);
    const double head_height = 2.0 * fixture::mesh_half_extent(head).y() * static_cast<double>(head->scale.y);
    const double head_middle = static_cast<double>(head->position.y);

    return drawn_profile{true,
                         at->normalized(),
                         stem->visible ? stem_radius : 0.0,
                         stem->visible ? stem_height : 0.0,
                         head_middle - head_height / 2.0,
                         head_middle + head_height / 2.0,
                         fixture::mesh_half_extent(head).x() * static_cast<double>(head->scale.x)};
}

// The radius the arrow occupies at that station along the ray it stands on, and nothing where it has
// no material there. The head's radius falls from its base to nothing at the tip.
double radius_at(const drawn_profile &drawn, double station)
{
    if(!drawn.material || station < 0.0 || station > drawn.head_top)
        return 0.0;

    const double stem = station <= drawn.stem_top ? drawn.stem_radius : 0.0;
    if(station < drawn.head_bottom || drawn.head_top <= drawn.head_bottom)
        return stem;

    return std::max(stem, drawn.head_radius * (drawn.head_top - station) / (drawn.head_top - drawn.head_bottom));
}

// Stations enough of them along the shorter arrow that the head's taper is resolved to well under a
// thousandth of what it spans, which is what the answer below is accurate to.
constexpr std::size_t stations_along = 4000;

// Whether one drawn arrow stands wholly within another: at every station along the ray it stands on
// where it has material, the other has material there too and is at least as wide. Two arrows placed
// along directions that do not agree share no station beyond the origin, and an arrow with no
// material anywhere stands within nothing. This is a containment check between two solids of
// revolution on one ray, not a reading of what a camera sees.
bool wholly_within(const drawn_profile &inside, const drawn_profile &around)
{
    if(!inside.material || !around.material || (inside.along - around.along).norm() > 1.0e-5)
        return false;

    for(std::size_t step = 0; step <= stations_along; ++step)
    {
        const double station = inside.head_top * static_cast<double>(step) / static_cast<double>(stations_along);

        if(radius_at(inside, station) > radius_at(around, station))
            return false;
    }

    return true;
}

bool hidden_in(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed, std::size_t inside, std::size_t around)
{
    return wholly_within(profile_of(headless, composed, inside), profile_of(headless, composed, around));
}

void neither_arrow_hides_the_other(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed, const std::vector<const char *> &angles)
{
    for(const char *typed : angles)
    {
        INFO(typed);
        driven_to(composed, typed);

        CHECK_FALSE(hidden_in(headless, composed, coordinate_object, axis_object));
        CHECK_FALSE(hidden_in(headless, composed, axis_object, coordinate_object));
    }
}

bool arrow_drawn(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed, std::size_t index)
{
    return fixture::drawn(fixture::first_mesh_under(*headless.scene, placed_in(composed).name_of(index), "head"));
}

// The whole of what the frame draws is the axes node the stencil hangs under it, the frame carrying
// no body of its own.
threepp::Object3D *frame_axes(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed)
{
    threepp::Object3D *found = nullptr;
    threepp::Object3D *node  = headless.scene->getObjectByName<threepp::Object3D>(std::string(placed_in(composed).name_of(rotation_axis_window::frame_object)));
    if(node == nullptr)
        return nullptr;

    node->traverse(
            [&found](threepp::Object3D &at)
            {
                if(found == nullptr && at.name == "axes")
                    found = &at;
            });

    return found;
}

// Built in the frame the stencil's poses are written in under an object that is never placed, so the
// points are read out of the buffer they were built in.
std::vector<Eigen::Vector3d> drawn_arc(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed, std::size_t index)
{
    return fixture::line_points(*headless.scene, placed_in(composed).name_of(index));
}

// Where the end of one of the frame's own arrows stands, in that same frame.
Eigen::Vector3d tip_of(const std::shared_ptr<scene::preset> &composed, const Eigen::Vector3d &carried)
{
    const transform put = placed_in(composed).world_pose(rotation_axis_window::frame_object);

    return put.topLeftCorner<3, 3>() * carried + put.block<3, 1>(0, 3);
}

// The perpendicular distance from a point to the axis ray, which in this scenario runs through the
// origin.
double off_axis(const Eigen::Vector3d &at, const Eigen::Vector3d &axis)
{
    return (at - at.dot(axis) * axis).norm();
}

// The furthest a point of the travelled arc can stand from the polyline drawn through samples of it:
// the sagitta of one sampled step, which the drawn geometry and the commanded angle together say,
// floored at the tolerance a point read back out of a float buffer is comparable at.
double arc_resolution(const std::vector<Eigen::Vector3d> &arc, const Eigen::Vector3d &axis, double angle_radians)
{
    const double step = std::abs(angle_radians) / static_cast<double>(arc.size() - 1u);

    return std::max(off_axis(arc.front(), axis) * (1.0 - std::cos(0.5 * step)), 1.0e-6);
}

// The turn taken about the axis from the arc's first point to its last, signed the way the axis is.
double subtended(const std::vector<Eigen::Vector3d> &arc, const Eigen::Vector3d &axis)
{
    const Eigen::Vector3d first = arc.front() - arc.front().dot(axis) * axis;
    const Eigen::Vector3d last  = arc.back() - arc.back().dot(axis) * axis;

    return std::atan2(axis.dot(first.cross(last)), first.dot(last));
}

// What says an arc is the one its arrow has travelled is that the arrow's end stands on it, within
// the sagitta of one sampled step, at whatever the controls now hold.
void every_arrow_stands_on_its_arc(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed)
{
    const Eigen::Vector3d axis = about(composed);
    const auto commanded       = static_cast<double>(panel_of(composed).state().angle_radians);

    for(const traced &subject : arcs)
    {
        INFO(subject.object);

        const std::vector<Eigen::Vector3d> arc = drawn_arc(headless, composed, subject.object);

        REQUIRE(arc.size() > 2u);
        CHECK(fixture::from_curve(arc, tip_of(composed, subject.carried)) <= arc_resolution(arc, axis, commanded));
    }
}

// An arc traced by one of the frame's arrows stands off the axis by the length of that arrow times the
// sine of the angle between the two, so every one of the three radii moves when the axis does.
void every_arc_stands_at_the_radius_the_axis_implies(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed)
{
    const Eigen::Vector3d axis = about(composed);

    for(const traced &subject : arcs)
    {
        INFO(subject.object);

        const std::vector<Eigen::Vector3d> arc = drawn_arc(headless, composed, subject.object);
        const double leaning                   = std::clamp(subject.carried.normalized().dot(axis), -1.0, 1.0);
        const double implied                   = subject.carried.norm() * std::sin(std::acos(leaning));

        REQUIRE(arc.size() > 2u);
        for(const Eigen::Vector3d &at : arc)
            CHECK(is_approx_equal(off_axis(at, axis), implied, 1.0e-5));
    }
}

bool every_arc_drawn(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed)
{
    for(const traced &subject : arcs)
        if(!fixture::drawn(fixture::first_line_under(*headless.scene, placed_in(composed).name_of(subject.object))))
            return false;

    return true;
}

struct standing_drawings
{
    bool axis;
    bool coordinate;
    bool arc;
    bool frame;
};

standing_drawings drawings_of(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed)
{
    return standing_drawings{arrow_drawn(headless, composed, axis_object), arrow_drawn(headless, composed, coordinate_object), every_arc_drawn(headless, composed),
                             fixture::drawn(frame_axes(headless, composed))};
}

bool every_drawing_stands(const standing_drawings &standing)
{
    return standing.axis && standing.coordinate && standing.arc && standing.frame;
}

// Which drawing a switch is supposed to take away, so the same four subjects are read after every
// press and a switch reaching past its own is what fails. A switch over the two arrows also carries
// the pair standing on one ray: a switch over a drawing wholly inside another takes away nothing a
// person could see. The arc switch builds no geometry at all when it is off and the frame switch acts
// where there is no second drawing to stand inside, so neither carries that pair.
struct switched
{
    const char *label;
    bool standing_drawings::*taken;
    std::optional<std::pair<std::size_t, std::size_t>> proud_of;
};

const std::vector<switched> switches{{"Unit axis", &standing_drawings::axis, std::make_pair(axis_object, coordinate_object)},
                                     {"Coordinate vector", &standing_drawings::coordinate, std::make_pair(coordinate_object, axis_object)},
                                     {"Traversed arc", &standing_drawings::arc, std::nullopt},
                                     {"Frame", &standing_drawings::frame, std::nullopt}};

struct commanded_angle
{
    const char *typed;
    double radians;
};

const std::vector<commanded_angle> rising{{"0.500", 0.5}, {"1.000", 1.0}, {"2.000", 2.0}, {"3.000", 3.0}};

std::vector<double> reaches_over_rising(fixture::stage &headless, const std::shared_ptr<scene::preset> &composed, std::size_t index)
{
    std::vector<double> reaches;
    for(const commanded_angle &commanded : rising)
    {
        INFO(commanded.typed);
        driven_to(composed, commanded.typed);

        REQUIRE(is_approx_equal(static_cast<double>(panel_of(composed).state().angle_radians), commanded.radians, 1.0e-5));

        reaches.push_back(reach_of(headless, composed, index));
    }

    return reaches;
}

}

TEST_CASE("the_composed_scenario_draws_its_panel_and_leaves_the_scene_as_it_found_it")
{
    fixture::stage headless;
    const std::size_t before = headless.descendants();

    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    REQUIRE(headless.descendants() > before);
    REQUIRE(fixture::composed_windows(composed) == std::vector<std::string>{"Rotation"});

    fixture::each_window_opens_one_panel(composed);

    composed->tear_down();

    CHECK(headless.descendants() == before);
}

TEST_CASE("the_composition_opens_at_a_unit_axis_off_every_coordinate_direction_and_at_one_radian")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    const rotation_axis_window::settings shown    = panel_of(composed).state();
    const Eigen::Vector3d spelled                 = Eigen::Vector3d{1.0, 1.0, 1.0}.normalized();

    CHECK(shown.angle_radians == 1.f);
    CHECK(is_approx_equal(static_cast<double>(shown.direction.norm()), 1.0, 1.0e-6));
    CHECK(is_approx_equal((shown.direction.cast<double>() - spelled).norm(), 0.0, 1.0e-6));

    for(Eigen::Index at = 0; at < 3; ++at)
    {
        INFO(at);
        CHECK(std::abs(shown.direction[at]) > 1.0e-3f);
    }
}

TEST_CASE("the_composition_draws_the_frame_at_the_origin_and_the_two_arrows_and_three_arcs_beside_it")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    frame_stencil &body                           = placed_in(composed);

    REQUIRE(body.count() == 6u);
    CHECK(body.name_of(rotation_axis_window::frame_object) == std::string("Frame"));
    CHECK(body.name_of(axis_object) == std::string("Axis"));
    CHECK(body.name_of(coordinate_object) == std::string("Coordinate"));
    CHECK(body.name_of(arc_x_object) == std::string("Arc x"));
    CHECK(body.name_of(arc_y_object) == std::string("Arc y"));
    CHECK(body.name_of(arc_z_object) == std::string("Arc z"));
    CHECK(body.axes_shown(rotation_axis_window::frame_object));
    CHECK_FALSE(body.axes_shown(axis_object));
    CHECK_FALSE(body.axes_shown(coordinate_object));

    for(const traced &subject : arcs)
    {
        INFO(subject.object);
        CHECK_FALSE(body.axes_shown(subject.object));
    }

    CHECK(body.fixed_frame_name().empty());
    CHECK(is_approx_equal(standing(composed).block<3, 1>(0, 3).norm(), 0.0, 1.0e-9));
}

TEST_CASE("the_angle_turns_the_composed_frame_by_the_exponential_the_composition_bound")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    const Eigen::Vector3d axis                    = about(composed);
    const transform start                         = transform::Identity();

    CHECK(is_approx_equal(standing(composed), turned(axis, 1.0, start), 1.0e-6));

    for(const char *angle : {"2.500", "-1.750", "0.000"})
    {
        INFO(angle);
        driven_to(composed, angle);

        const auto commanded = static_cast<double>(panel_of(composed).state().angle_radians);

        CHECK(is_approx_equal(standing(composed), turned(axis, commanded, start), 1.0e-5));
    }

    // A zero angle is the identity rotation, which is where the composition placed the frame.
    CHECK(panel_of(composed).state().angle_radians == 0.f);
    CHECK(is_approx_equal(standing(composed), start, 1.0e-9));
}

// Both arrows are built the same way and their heads are proportioned the same way, so what stands
// the two heads at one distance from the origin is the two drawn lengths being equal. That is the
// invariant the coincidence below rests on and it cannot be read off the assertion.
TEST_CASE("the_axis_arrow_is_drawn_at_the_length_the_frames_own_arrows_are_drawn_at")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    CHECK(is_approx_equal(reach_of(headless, composed, axis_object), axes_settings{}.axis_length, 1.0e-5));

    driven_to(composed, "1.000");

    const std::optional<Eigen::Vector3d> named = arrow_head(headless, composed, axis_object);
    const std::optional<Eigen::Vector3d> ahead = arrow_head(headless, composed, coordinate_object);

    REQUIRE(named.has_value());
    REQUIRE(ahead.has_value());
    CHECK(is_approx_equal(ahead->norm(), named->norm(), 1.0e-5));
}

// Both arrows are drawn through the one length: the axis arrow at it and the coordinate arrow at it
// times the angle. What the two reach therefore stands in the ratio of the angle itself, whatever
// that length is.
TEST_CASE("the_coordinate_arrow_reaches_the_angle_times_what_the_axis_arrow_reaches")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    const std::vector<double> reaches             = reaches_over_rising(headless, composed, coordinate_object);
    const double against                          = reach_of(headless, composed, axis_object);

    REQUIRE(against > 0.0);

    for(std::size_t step = 0; step < rising.size(); ++step)
    {
        INFO(rising[step].typed);
        CHECK(is_approx_equal(reaches[step] / against, rising[step].radians, 1.0e-5));
    }
}

TEST_CASE("the_unit_arrow_reaches_the_same_distance_however_far_the_angle_is_driven")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    const std::vector<double> reaches             = reaches_over_rising(headless, composed, axis_object);

    for(std::size_t step = 0; step < rising.size(); ++step)
    {
        INFO(rising[step].typed);
        CHECK(is_approx_equal(reaches[step], reaches.front(), 1.0e-6));
    }

    // The angle now stands at the last of those, where what the arrow reading the angle reaches is
    // that many times what the arrow reading the axis reaches.
    CHECK(is_approx_equal(reach_of(headless, composed, coordinate_object), rising.back().radians * reaches.back(), 1.0e-5));
}

// At no turn at all the rotation is the identity and the frame has travelled nowhere, so the arcs go
// the way the coordinate arrow does rather than being padded to something drawable.
TEST_CASE("at_no_angle_at_all_neither_the_coordinate_arrow_nor_any_arc_is_drawn_and_the_unit_arrow_is")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    REQUIRE(every_drawing_stands(drawings_of(headless, composed)));

    driven_to(composed, "0.000");

    REQUIRE(panel_of(composed).state().angle_radians == 0.f);

    const standing_drawings collapsed = drawings_of(headless, composed);

    CHECK_FALSE(collapsed.coordinate);
    CHECK_FALSE(collapsed.arc);
    CHECK(collapsed.axis);
    CHECK(collapsed.frame);

    for(const traced &subject : arcs)
    {
        INFO(subject.object);
        CHECK(fixture::first_line_under(*headless.scene, placed_in(composed).name_of(subject.object)) == nullptr);
    }

    driven_to(composed, "1.000");

    CHECK(every_drawing_stands(drawings_of(headless, composed)));
}

// The state where the coordinate vector and every arc have collapsed and the arrow standing for the
// axis alone stands is one press away, whatever the angle was driven to.
TEST_CASE("a_control_beside_the_angle_collapses_the_coordinate_arrow_and_every_arc")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    driven_to(composed, "2.500");

    REQUIRE(panel_of(composed).state().angle_radians != 0.f);
    REQUIRE(every_drawing_stands(drawings_of(headless, composed)));

    press_reset(composed);

    CHECK(panel_of(composed).state().angle_radians == 0.f);

    const standing_drawings collapsed = drawings_of(headless, composed);

    CHECK_FALSE(collapsed.coordinate);
    CHECK_FALSE(collapsed.arc);
    CHECK(collapsed.axis);
    CHECK(collapsed.frame);
}

TEST_CASE("neither_arrow_is_wholly_within_the_other_but_where_their_drawn_lengths_coincide")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    // An arrow that is not drawn at all is absent rather than standing inside anything, and a switch
    // is asked about only one of those, so the angle of zero stands among the angles tried.
    neither_arrow_hides_the_other(headless, composed, {"0.000", "0.500", "0.900", "1.100", "2.000", "3.000", "-1.000", "-3.000"});

    // The axis arrow is drawn at the length the frame's own arrows are drawn at and the coordinate
    // arrow at that length times the angle, so the two drawn lengths agree at one radian and at no
    // other angle. There the thinner stands wholly inside the thicker, and which of the two that is
    // is what the girth ordering decides.
    driven_to(composed, "1.000");

    REQUIRE(is_approx_equal(reach_of(headless, composed, axis_object), axes_settings{}.axis_length, 1.0e-5));
    CHECK(is_approx_equal(reach_of(headless, composed, coordinate_object), axes_settings{}.axis_length, 1.0e-5));
    CHECK(hidden_in(headless, composed, coordinate_object, axis_object));
    CHECK_FALSE(hidden_in(headless, composed, axis_object, coordinate_object));

    typed_direction(composed, "0", "0", "1");
    neither_arrow_hides_the_other(headless, composed, {"0.500", "2.000", "-2.000"});
    driven_to(composed, "1.000");

    CHECK(hidden_in(headless, composed, coordinate_object, axis_object));
}

TEST_CASE("a_negative_angle_puts_the_coordinate_arrow_on_the_far_side_of_the_origin")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    driven_to(composed, "1.500");

    const Eigen::Vector3d named = along_of(headless, composed, axis_object);
    const Eigen::Vector3d ahead = along_of(headless, composed, coordinate_object);
    const double reached        = reach_of(headless, composed, coordinate_object);

    driven_to(composed, "-1.500");

    const Eigen::Vector3d behind = along_of(headless, composed, coordinate_object);

    CHECK(is_approx_equal((ahead - named).norm(), 0.0, 1.0e-5));
    CHECK(is_approx_equal((behind + named).norm(), 0.0, 1.0e-5));
    CHECK(is_approx_equal(reach_of(headless, composed, coordinate_object), reached, 1.0e-5));
    CHECK(is_approx_equal((along_of(headless, composed, axis_object) - named).norm(), 0.0, 1.0e-5));
}

TEST_CASE("a_direction_typed_into_the_controls_carries_both_arrows_onto_it")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    REQUIRE(is_approx_equal((along_of(headless, composed, axis_object) - about(composed)).norm(), 0.0, 1.0e-5));

    typed_direction(composed, "1", "0", "0");

    const Eigen::Vector3d named = about(composed);

    REQUIRE(is_approx_equal((named - Eigen::Vector3d::UnitX()).norm(), 0.0, 1.0e-6));
    CHECK(is_approx_equal((along_of(headless, composed, axis_object) - named).norm(), 0.0, 1.0e-5));
    CHECK(is_approx_equal((along_of(headless, composed, coordinate_object) - named).norm(), 0.0, 1.0e-5));
}

TEST_CASE("each_switch_hides_its_own_drawing_and_leaves_the_other_three_standing")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    // The two arrows are the same length at one radian and the thinner then stands wholly inside the
    // thicker, so the angle is driven off that one station before any switch is pressed.
    driven_to(composed, "2.000");

    REQUIRE(every_drawing_stands(drawings_of(headless, composed)));

    for(const switched &pressed : switches)
    {
        INFO(pressed.label);

        if(pressed.proud_of)
            REQUIRE_FALSE(hidden_in(headless, composed, pressed.proud_of->first, pressed.proud_of->second));

        fixture::press_on(panel_of(composed), pressed.label);

        const standing_drawings after = drawings_of(headless, composed);
        for(const switched &subject : switches)
        {
            INFO(subject.label);
            CHECK(after.*subject.taken == (subject.taken != pressed.taken));
        }

        fixture::press_on(panel_of(composed), pressed.label);

        CHECK(every_drawing_stands(drawings_of(headless, composed)));
    }
}

// Hiding is a view of a drawing rather than a second spelling of its length, so a drawing shown again
// stands where it stood.
TEST_CASE("a_switch_that_hides_a_drawing_and_shows_it_again_moves_nothing")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());
    const std::optional<Eigen::Vector3d> before   = arrow_head(headless, composed, coordinate_object);

    REQUIRE(before.has_value());

    fixture::press_on(panel_of(composed), "Coordinate vector");
    fixture::press_on(panel_of(composed), "Coordinate vector");

    const std::optional<Eigen::Vector3d> after = arrow_head(headless, composed, coordinate_object);

    REQUIRE(after.has_value());
    CHECK(arrow_drawn(headless, composed, coordinate_object));
    CHECK(is_approx_equal((*after - *before).norm(), 0.0, 1.0e-9));
}

// The radius each arc is traced at is the length of its arrow times the sine of the angle that arrow
// makes with the axis, so an arc rebuilt on the angle alone is a different line from the one the axis
// now implies.
TEST_CASE("a_direction_typed_into_the_controls_redraws_every_arc")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    std::vector<std::vector<Eigen::Vector3d>> before;
    for(const traced &subject : arcs)
    {
        before.push_back(drawn_arc(headless, composed, subject.object));

        REQUIRE(before.back().size() > 2u);
    }

    typed_direction(composed, "0", "0", "1");

    REQUIRE(is_approx_equal((about(composed) - Eigen::Vector3d::UnitZ()).norm(), 0.0, 1.0e-6));

    for(std::size_t at = 0; at < arcs.size(); ++at)
    {
        INFO(arcs[at].object);
        CHECK_FALSE(fixture::same_line(drawn_arc(headless, composed, arcs[at].object), before[at]));
    }
}

TEST_CASE("the_frames_arrow_ends_stand_on_their_arcs_at_every_angle_after_the_axis_has_been_moved")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    every_arrow_stands_on_its_arc(headless, composed);

    typed_direction(composed, "0", "0", "1");

    REQUIRE(is_approx_equal((about(composed) - Eigen::Vector3d::UnitZ()).norm(), 0.0, 1.0e-6));

    for(const char *angle : {"0.500", "1.000", "-2.500", "3.000"})
    {
        INFO(angle);
        driven_to(composed, angle);

        REQUIRE(panel_of(composed).state().angle_radians != 0.f);

        every_arrow_stands_on_its_arc(headless, composed);
    }
}

TEST_CASE("each_arc_stands_off_the_axis_at_the_radius_the_named_axis_implies")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    every_arc_stands_at_the_radius_the_axis_implies(headless, composed);

    typed_direction(composed, "0", "0", "1");

    REQUIRE(is_approx_equal((about(composed) - Eigen::Vector3d::UnitZ()).norm(), 0.0, 1.0e-6));

    every_arc_stands_at_the_radius_the_axis_implies(headless, composed);

    // The arrow standing along the axis is carried nowhere by the turn, so the arc it traces has
    // collapsed onto the axis while the two across it stand a whole arrow's length off it.
    for(const Eigen::Vector3d &at : drawn_arc(headless, composed, arc_z_object))
        CHECK(is_approx_equal(off_axis(at, Eigen::Vector3d::UnitZ()), 0.0, 1.0e-6));

    for(const std::size_t across : {arc_x_object, arc_y_object})
    {
        INFO(across);
        CHECK(is_approx_equal(off_axis(drawn_arc(headless, composed, across).front(), Eigen::Vector3d::UnitZ()), axes_settings{}.axis_length, 1.0e-5));
    }
}

// The extent is the second reading of the angle the coordinate arrow's length is the first of: what
// is drawn is the turn already taken, not the whole turn the control could reach.
TEST_CASE("each_arc_subtends_the_angle_the_control_holds_and_no_more")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    for(const char *angle : {"0.500", "1.000", "-2.500", "3.000"})
    {
        INFO(angle);
        driven_to(composed, angle);

        const Eigen::Vector3d axis = about(composed);
        const auto commanded       = static_cast<double>(panel_of(composed).state().angle_radians);

        for(const traced &subject : arcs)
        {
            INFO(subject.object);

            const std::vector<Eigen::Vector3d> arc = drawn_arc(headless, composed, subject.object);

            REQUIRE(arc.size() > 2u);
            CHECK(is_approx_equal(subtended(arc, axis), commanded, 1.0e-4));
        }
    }
}

TEST_CASE("the_arc_grows_out_of_where_it_stood_as_the_angle_rises_and_shrinks_back_as_it_falls")
{
    fixture::stage headless;
    const std::shared_ptr<scene::preset> composed = opened(headless, baseline());

    driven_to(composed, "1.000");

    const std::vector<Eigen::Vector3d> shorter = drawn_arc(headless, composed, arc_x_object);

    driven_to(composed, "3.000");

    const std::vector<Eigen::Vector3d> longer = drawn_arc(headless, composed, arc_x_object);

    REQUIRE(shorter.size() > 2u);
    CHECK(shorter.size() < longer.size());
    CHECK(fixture::from_curve(longer, shorter.back()) <= arc_resolution(longer, about(composed), 3.0));
    CHECK(is_approx_equal((longer.front() - shorter.front()).norm(), 0.0, 1.0e-6));

    driven_to(composed, "1.000");

    CHECK(fixture::same_line(drawn_arc(headless, composed, arc_x_object), shorter));
}

// The one substitution that catches an arc drawn from a formula: a frame turned one way and arcs
// drawn another would be two answers to one question.
TEST_CASE("the_arcs_and_the_frame_both_follow_the_exponential_the_composition_bound")
{
    fixture::stage headless;
    capabilities motions                          = baseline();
    motions.screw.matrix_exponential_so3          = &turned_further;
    const std::shared_ptr<scene::preset> composed = opened(headless, motions);
    const Eigen::Vector3d axis                    = about(composed);
    const auto commanded                          = static_cast<double>(panel_of(composed).state().angle_radians);

    REQUIRE(commanded != 0.0);
    CHECK(is_approx_equal(standing(composed), turned(axis, commanded + 1.0, transform::Identity()), 1.0e-6));

    for(const traced &subject : arcs)
    {
        INFO(subject.object);

        const std::vector<Eigen::Vector3d> arc = drawn_arc(headless, composed, subject.object);

        REQUIRE(arc.size() > 2u);
        CHECK(is_approx_equal((arc.back() - tip_of(composed, subject.carried)).norm(), 0.0, 1.0e-5));
        CHECK(is_approx_equal((arc.back() - matrix_exponential_so3(axis, commanded + 1.0) * subject.carried).norm(), 0.0, 1.0e-5));
    }
}
