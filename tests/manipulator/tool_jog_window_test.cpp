#include "window_stage.h"

#include "praxis/manipulator/option_widgets.h"
#include "praxis/manipulator/tool_jog_window.h"

#include "praxis/scene/widgets.h"

#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/axis_order.h"
#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_approx.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>

#include <Eigen/Core>

#include <imgui.h>

#include <memory>
#include <vector>
#include <cstddef>
#include <stdexcept>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;
using Catch::Matchers::Message;

namespace {

using mode = control_mode;

const rigid_motion::frame_ops reference = rigid_motion::baseline().frame;

const char *const position_labels[3] = {"X", "Y", "Z"};
const char *const angle_labels[3]    = {"A", "B", "C"};

// No two of them equal and none a right angle or a straight one, so a pose carried the wrong way
// through the seam lands where none of them is.
const Eigen::Vector3d chosen_euler_degrees{37.0, 52.0, -19.0};
const Eigen::Vector3d chosen_position{0.25, -0.4, 0.7};
const rotation chosen_orientation = reference.rotation_matrix_from_euler(chosen_euler_degrees * radians_per_degree, axis_order::zyx);

// Inside each slider's own range, neither a whole turn nor a component of the published pose, so a
// value reaching the seam names the control it was entered at.
constexpr const char *typed_angle  = "24";
constexpr const char *typed_offset = "0.6";
constexpr double jog_angle_degrees = 24.0;
constexpr double jog_offset        = 0.6;

// Both ends hold the jog in single precision, so a component reaching the seam is the component
// behind it to within one float step of it.
constexpr double float_step = 1.0e-4;

// The entry the order selector's list opens on, which is not the order an untouched shared pose
// composes its angles in.
constexpr axis_order first_offered = axis_order::xyz;

struct displacement
{
    transform start;
    Eigen::Vector3d offset;
    rotation turned;
};

std::vector<displacement> jogged;

expected<joint_vector, refusal> recorded_jog(const kinematics &, const transform &start, const Eigen::Vector3d &offset, const rotation &turned, const joint_vector &j0)
{
    jogged.push_back(displacement{start, offset, turned});

    return j0;
}

composed_arm jogging(praxis::scheduler::scheduler &loop)
{
    return compose(loop, motion_ops{.tool_frame_displace = &recorded_jog}, rigid_motion::baseline().screw);
}

arm_snapshot chosen_snapshot()
{
    return at_rest(configuration(0.0, 0.0), chosen_position, chosen_orientation);
}

arm_snapshot poseless_snapshot()
{
    return at_rest(configuration(0.0, 0.0), praxis::unexpected(refusal::no_solution), praxis::unexpected(refusal::no_solution));
}

std::shared_ptr<edited_pose> holding(const Eigen::Vector3d &position, const Eigen::Vector3d &euler_degrees)
{
    auto held           = std::make_shared<edited_pose>();
    held->position      = position.cast<float>();
    held->euler_degrees = euler_degrees.cast<float>();

    return held;
}

drawing over_both(tool_jog_window &first, tool_jog_window &second, const char *focused)
{
    return [&first, &second, focused]
    {
        first.render();
        second.render();
        ImGui::SetWindowFocus(focused);
    };
}

// The panel a tool jog window draws, written out here rather than read off the window: which widget
// stands where and what it is labeled is the case's own claim.
void draw_reference(const edited_pose &edited)
{
    option_cycle<mode, 2> selected(mode::preview, {mode::preview, mode::simulation}, {"Preview", "Simulation"});
    edited_pose shown       = edited;
    Eigen::Vector3f offset  = Eigen::Vector3f::Zero();
    Eigen::Vector3f turning = Eigen::Vector3f::Zero();

    ImGui::Begin("Tool frame jog");
    render_option_cycle("Control mode", selected);
    ImGui::InputFloat3("XYZ", shown.position.data());
    ImGui::InputFloat3("ABC", shown.euler_degrees.data());
    scene::render_enum_selection("Euler order", shown.order, axis_order_labels());
    static_cast<void>(ImGui::Button("Reset"));
    scene::render_float3_slider(offset, position_labels, -1.f, 1.f);
    scene::render_float3_slider(turning, angle_labels, -180.f, 180.f);
    ImGui::End();
}

// The angle sliders are the pane's last three rows and the offset sliders the three above them, so
// the last row is the third jog angle and three steps up from it is the third jog offset.
void enter_last_angle(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    type_at_cursor(frames, draw, typed_angle);
}

void enter_last_offset(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    for(int step = 0; step < 3; ++step)
        tap(frames, draw, ImGuiKey_UpArrow);
    type_at_cursor(frames, draw, typed_offset);
}

// The seeding control stands six rows above the pane's last, under the two input triples and the
// order selector.
void press_reset(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    for(int step = 0; step < 6; ++step)
        tap(frames, draw, ImGuiKey_UpArrow);
    tap(frames, draw, ImGuiKey_Space);
}

// The start pose's position row stands nine rows above the pane's last, and a row is entered at its
// leftmost, which is the first component of the triple.
void enter_start_position(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    for(int step = 0; step < 9; ++step)
        tap(frames, draw, ImGuiKey_UpArrow);
    type_at_cursor(frames, draw, typed_offset);
}

// The order selector stands seven rows above the pane's last and offers its entries in a popup of
// its own, which opens on its first entry.
void take_first_euler_order(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    for(int step = 0; step < 7; ++step)
        tap(frames, draw, ImGuiKey_UpArrow);
    tap(frames, draw, ImGuiKey_Space);
    tap(frames, draw, ImGuiKey_Space);
}

// A focus request naming the panel closes any popup standing over it, so the frames that drive the
// order selector's list draw the panel and ask for nothing.
drawing over_alone(tool_jog_window &panel)
{
    return [&panel] { panel.render(); };
}

void starts_at(const transform &recorded, const Eigen::Vector3d &position, const rotation &orientation)
{
    for(Eigen::Index axis = 0; axis < 3; ++axis)
        CHECK(recorded(axis, 3) == Catch::Approx(position[axis]).margin(float_step));
    CHECK(recorded.topLeftCorner<3, 3>().isApprox(orientation, float_step));
}

// A panel whose reset landed is indistinguishable from one where the jog was never entered, and the
// nav cursor stands on the same control at the end of both sequences, so the two frames differ only
// in what the controls carry.
std::size_t reset_over(const arm_snapshot &seen, bool entering)
{
    const std::shared_ptr<arm_publisher> published = publishing(seen);
    tool_jog_window panel("Tool frame jog", published->reader(), std::weak_ptr<owned_arm>(), reference, std::make_shared<edited_pose>(), {mode::preview});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    if(entering)
        enter_last_offset(frames, draw);
    press_reset(frames, draw);

    return frames.signature();
}

rotation turned_by(double third_angle_degrees, axis_order order)
{
    return reference.rotation_matrix_from_euler(Eigen::Vector3d{0.0, 0.0, third_angle_degrees * radians_per_degree}, order);
}

}

TEST_CASE("a tool jog window built over an arm that has published nothing, or over no pose, refuses and names both ends", "[manipulator]")
{
    arm_publisher unheld;
    const std::shared_ptr<arm_publisher> published = publishing(chosen_snapshot());

    REQUIRE_THROWS_MATCHES(tool_jog_window("Tool frame jog", unheld.reader(), std::weak_ptr<owned_arm>(), reference, std::make_shared<edited_pose>()), std::invalid_argument,
                           Message("praxis: the tool jog window was given no published arm state to hold"));
    REQUIRE_THROWS_MATCHES(tool_jog_window("Tool frame jog", published->reader(), std::weak_ptr<owned_arm>(), reference, nullptr), std::invalid_argument,
                           Message("praxis: the tool jog window was given no shared pose to hold"));
}

TEST_CASE("a tool jog panel left with no publication draws one panel stating that", "[manipulator]")
{
    const std::shared_ptr<arm_publisher> published = publishing(chosen_snapshot());
    tool_jog_window panel("Tool frame jog", published->reader(), std::weak_ptr<owned_arm>(), reference, std::make_shared<edited_pose>());
    published->publish(nullptr);

    CHECK(geometry_of([&panel] { panel.render(); }) == stating_absence("Tool frame jog"));
}

TEST_CASE("a tool jog window draws the start pose, the order, the seeding control and the two jog rows", "[manipulator][controls]")
{
    const std::shared_ptr<arm_publisher> published = publishing(chosen_snapshot());
    const std::shared_ptr<edited_pose> held        = holding(chosen_position, chosen_euler_degrees);
    tool_jog_window panel("Tool frame jog", published->reader(), std::weak_ptr<owned_arm>(), reference, held, {mode::preview});

    edited_pose elsewhere = *held;
    elsewhere.order       = axis_order::xyz;

    CHECK(geometry_of([&panel] { panel.render(); }) == geometry_of([&held] { draw_reference(*held); }));
    CHECK(geometry_of([&panel] { panel.render(); }) != geometry_of([&elsewhere] { draw_reference(elsewhere); }));
}

TEST_CASE("an angle entered at a tool jog slider is previewed as a jog about the shared pose", "[manipulator][controls]")
{
    jogged.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed               = jogging(loop);
    const std::shared_ptr<edited_pose> held = holding(chosen_position, chosen_euler_degrees);
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    tool_jog_window panel("Tool frame jog", placed.seen, placed.owned, reference, held, {mode::preview});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    enter_last_angle(frames, draw);
    static_cast<void>(loop.drain());

    REQUIRE(!jogged.empty());
    starts_at(jogged.back().start, chosen_position, chosen_orientation);
    CHECK(jogged.back().offset.isZero(float_step));
    CHECK(jogged.back().turned.isApprox(turned_by(jog_angle_degrees, axis_order::zyx), float_step));
}

TEST_CASE("an offset entered at a tool jog slider is previewed as the offset the jog carries", "[manipulator][controls]")
{
    jogged.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed = jogging(loop);
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    tool_jog_window panel("Tool frame jog", placed.seen, placed.owned, reference, holding(chosen_position, chosen_euler_degrees), {mode::preview});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    enter_last_offset(frames, draw);
    static_cast<void>(loop.drain());

    REQUIRE(!jogged.empty());
    CHECK(jogged.back().offset.isApprox(Eigen::Vector3d{0.0, 0.0, jog_offset}, float_step));
    CHECK(jogged.back().turned.isApprox(rotation::Identity(), float_step));
}

TEST_CASE("a tool jog window in simulation issues nothing at all, because the group is preview only", "[manipulator][controls]")
{
    jogged.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed = jogging(loop);
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    tool_jog_window panel("Tool frame jog", placed.seen, placed.owned, reference, holding(chosen_position, chosen_euler_degrees), {mode::simulation});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    enter_last_angle(frames, draw);
    enter_last_offset(frames, draw);
    static_cast<void>(loop.drain());

    CHECK(jogged.empty());
}

TEST_CASE("a tool jog window's seeding control reaches the shared pose, and only over a published tool pose", "[manipulator][controls]")
{
    const bool carried                             = GENERATE(true, false);
    const std::shared_ptr<arm_publisher> published = publishing(carried ? chosen_snapshot() : poseless_snapshot());
    const std::shared_ptr<edited_pose> held        = std::make_shared<edited_pose>();
    tool_jog_window panel("Tool frame jog", published->reader(), std::weak_ptr<owned_arm>(), reference, held, {mode::preview});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    press_reset(frames, draw);

    const Eigen::Vector3d seeded = carried ? chosen_position : Eigen::Vector3d::Zero();
    const Eigen::Vector3d angles = carried ? chosen_euler_degrees : Eigen::Vector3d::Zero();
    CHECK(held->position.cast<double>().isApprox(seeded, float_step));
    CHECK(held->euler_degrees.cast<double>().isApprox(angles, float_step));
}

TEST_CASE("a tool jog window's seeding control clears its own jog, and only over a published tool pose", "[manipulator][controls]")
{
    CHECK(reset_over(chosen_snapshot(), true) == reset_over(chosen_snapshot(), false));
    CHECK(reset_over(poseless_snapshot(), true) != reset_over(poseless_snapshot(), false));
}

TEST_CASE("two tool jog windows over one shared pose read one pose and jog independently", "[manipulator][controls]")
{
    jogged.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed               = jogging(loop);
    const std::shared_ptr<edited_pose> held = std::make_shared<edited_pose>();
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    tool_jog_window seeding("Tool frame jog##1", placed.seen, placed.owned, reference, held, {mode::preview});
    tool_jog_window turning("Tool frame jog##2", placed.seen, placed.owned, reference, held, {mode::preview});

    imgui_frame frames;
    start_navigating(frames, over_both(seeding, turning, "Tool frame jog##1"));
    press_reset(frames, over_both(seeding, turning, "Tool frame jog##1"));
    enter_last_offset(frames, over_both(seeding, turning, "Tool frame jog##1"));
    enter_last_angle(frames, over_both(seeding, turning, "Tool frame jog##2"));
    static_cast<void>(loop.drain());

    REQUIRE(!jogged.empty());
    starts_at(jogged.back().start, chosen_position, chosen_orientation);
    CHECK(jogged.back().offset.isZero(float_step));
    CHECK(jogged.back().turned.isApprox(turned_by(jog_angle_degrees, axis_order::zyx), float_step));
}

TEST_CASE("a tool jog's own rotation is not reinterpreted by the order another panel set", "[manipulator][controls]")
{
    jogged.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed                 = jogging(loop);
    const std::shared_ptr<edited_pose> shared = holding(chosen_position, Eigen::Vector3d::Zero());
    shared->order                             = axis_order::xyz;
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    tool_jog_window panel("Tool frame jog", placed.seen, placed.owned, reference, shared, {mode::preview});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    enter_last_angle(frames, draw);
    static_cast<void>(loop.drain());

    REQUIRE(!jogged.empty());
    CHECK(jogged.back().turned.isApprox(turned_by(jog_angle_degrees, axis_order::zyx), float_step));
    CHECK(!jogged.back().turned.isApprox(turned_by(jog_angle_degrees, axis_order::xyz), float_step));
}

TEST_CASE("a tool jog window in preview jogs from the start pose the orientation order it was moved to composes", "[manipulator][controls]")
{
    jogged.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed               = jogging(loop);
    const std::shared_ptr<edited_pose> held = holding(chosen_position, chosen_euler_degrees);
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    tool_jog_window panel("Tool frame jog", placed.seen, placed.owned, reference, held, {mode::preview});

    imgui_frame frames;
    const drawing draw = over_alone(panel);
    start_navigating(frames, draw);
    take_first_euler_order(frames, draw);
    static_cast<void>(loop.drain());

    REQUIRE(!jogged.empty());
    starts_at(jogged.back().start, chosen_position, reference.rotation_matrix_from_euler(chosen_euler_degrees * radians_per_degree, first_offered));
    CHECK(!jogged.back().start.topLeftCorner<3, 3>().isApprox(chosen_orientation, float_step));
    CHECK(jogged.back().offset.isZero(float_step));
    CHECK(jogged.back().turned.isApprox(rotation::Identity(), float_step));
}

TEST_CASE("a tool jog window in preview jogs from the start pose its position row was moved to", "[manipulator][controls]")
{
    jogged.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed               = jogging(loop);
    const std::shared_ptr<edited_pose> held = holding(chosen_position, chosen_euler_degrees);
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    tool_jog_window panel("Tool frame jog", placed.seen, placed.owned, reference, held, {mode::preview});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    enter_start_position(frames, draw);
    static_cast<void>(loop.drain());

    REQUIRE(!jogged.empty());
    starts_at(jogged.back().start, Eigen::Vector3d{jog_offset, chosen_position[1], chosen_position[2]}, chosen_orientation);
    CHECK(jogged.back().offset.isZero(float_step));
    CHECK(jogged.back().turned.isApprox(rotation::Identity(), float_step));
}
