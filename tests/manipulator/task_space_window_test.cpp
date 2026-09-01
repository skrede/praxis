#include "window_stage.h"

#include "praxis/manipulator/option_widgets.h"
#include "praxis/manipulator/task_space_window.h"

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
#include <stdexcept>
#include <functional>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;
using Catch::Matchers::Message;

namespace {

using shape = task_space_window::motion_shape;
using mode  = control_mode;

const rigid_motion::frame_ops reference = rigid_motion::baseline().frame;

const char *const position_labels[3] = {"X", "Y", "Z"};
const char *const angle_labels[3]    = {"A", "B", "C"};

// No two of them equal and none a right angle or a straight one, so a pose carried the wrong way
// through the seam lands where none of them is.
const Eigen::Vector3d chosen_euler_degrees{37.0, 52.0, -19.0};
const Eigen::Vector3d chosen_position{0.25, -0.4, 0.7};
const rotation chosen_orientation = reference.rotation_matrix_from_euler(chosen_euler_degrees * radians_per_degree, axis_order::zyx);

// Inside the slider's own range and not one of the published components, so a row carrying it cannot
// be mistaken for a row carrying the seeded pose.
constexpr const char *typed_offset = "0.6";
constexpr double slider_offset     = 0.6;

// Both ends hold the pose in single precision, so a component reaching the seam is the component
// behind it to within one float step of it.
constexpr double float_step = 1.0e-4;

// The entry the order selector's list opens on, which is not the order an untouched shared pose
// composes its angles in.
constexpr axis_order first_offered = axis_order::xyz;

std::vector<transform> resolved;
std::vector<transform> straight;

expected<joint_vector, refusal> recorded_pose(const kinematics &solver, const transform &pose, const joint_vector &j0)
{
    resolved.push_back(pose);

    return position_as_configuration(solver, pose, j0);
}

expected<transform, refusal> recorded_decoupled(const transform &start, const transform &end, double s)
{
    straight.push_back(end);

    return interpolated(start, end, s);
}

composed_arm composing(praxis::scheduler::scheduler &loop)
{
    const trajectory::path_ops along{.joint_straight_line = &straight_line, .decoupled = &recorded_decoupled};
    const strand work    = *loop.make_strand();
    const auto driven    = std::make_shared<scene_robot>(two_joint_arm(robot_ops{}));
    const auto published = std::make_shared<arm_publisher>();
    const auto control   = std::make_shared<robot_controller>(*driven, motion_ops{.task_space_pose = &recorded_pose}, along, task_trajectory_ops{}, composing_time_scaling(),
                                                              trajectory::trajectory_ops{}, rigid_motion::baseline().screw);

    return composed_arm{published->reader(), published, std::make_shared<owned_arm>(work, work, driven, control, published)};
}

arm_snapshot chosen_snapshot()
{
    return at_rest(configuration(0.0, 0.0), chosen_position, chosen_orientation);
}

arm_snapshot poseless_snapshot()
{
    return at_rest(configuration(0.0, 0.0), unexpected(refusal::no_solution), unexpected(refusal::no_solution));
}

std::shared_ptr<edited_pose> holding(const Eigen::Vector3d &position, const Eigen::Vector3d &euler_degrees)
{
    auto held           = std::make_shared<edited_pose>();
    held->position      = position.cast<float>();
    held->euler_degrees = euler_degrees.cast<float>();

    return held;
}

drawing over_both(task_space_window &first, task_space_window &second, const char *focused)
{
    return [&first, &second, focused]
    {
        first.render();
        second.render();
        ImGui::SetWindowFocus(focused);
    };
}

// The two panels a task space window draws, written out here rather than read off the window: which
// widget stands where, what it is labeled and what the two cycles offer are the case's own claim.
void draw_slider_group(const edited_pose &edited)
{
    option_cycle<mode, 2> selected(mode::preview, {mode::preview, mode::simulation}, {"Preview", "Simulation"});
    option_cycle<shape, 2> trajectory(shape::ptp, {shape::ptp, shape::lin}, {"P2P", "LIN"});
    edited_pose shown = edited;

    ImGui::Begin("Task space");
    render_option_cycle("Control mode", selected);
    render_option_cycle("Trajectory", trajectory);
    scene::render_float3_slider(shown.position, position_labels, -1.f, 1.f);
    ImGui::NewLine();
    scene::render_float3_slider_with_reset(shown.euler_degrees, angle_labels, -180.f, 180.f);
    scene::render_enum_selection("Euler order", shown.order, axis_order_labels());
    static_cast<void>(ImGui::Button("Reset to current"));
    ImGui::End();
}

void draw_field_group(const edited_pose &edited)
{
    option_cycle<mode, 2> selected(mode::simulation, {mode::preview, mode::simulation}, {"Preview", "Simulation"});
    option_cycle<shape, 2> trajectory(shape::lin, {shape::ptp, shape::lin}, {"P2P", "LIN"});
    edited_pose shown = edited;

    ImGui::Begin("Task space");
    render_option_cycle("Control mode", selected);
    render_option_cycle("Trajectory", trajectory);
    scene::render_float3_inputs(shown.position, position_labels, 0.01f, 0.1f);
    ImGui::NewLine();
    render_euler_inputs("Euler order", shown.euler_degrees, shown.order, 0.01f, 0.1f);
    static_cast<void>(ImGui::Button("Reset to current"));
    ImGui::SameLine();
    static_cast<void>(ImGui::Button("Move"));
    ImGui::End();
}

// The seeding control is the leftmost widget of the pane's last row in either panel, and a row is
// entered at its leftmost; in simulation the move control stands beside it, one sideways step on.
void press_reset(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_Space);
}

void press_move(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_Space);
}

// The order selector stands one row above the pane's last and offers its entries in a popup of its
// own, which opens on its first entry.
void take_first_euler_order(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_UpArrow);
    tap(frames, draw, ImGuiKey_Space);
    tap(frames, draw, ImGuiKey_Space);
}

// A focus request naming the panel closes any popup standing over it, so the frames that drive the
// order selector's list draw the panel and ask for nothing.
drawing over_alone(task_space_window &panel)
{
    return [&panel] { panel.render(); };
}

// The first position control stands two rows under the pane's first, which the two cycles occupy.
void enter_first_offset(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_Home);
    tap(frames, draw, ImGuiKey_DownArrow);
    tap(frames, draw, ImGuiKey_DownArrow);
    type_at_cursor(frames, draw, typed_offset);
}

void reaches(const transform &recorded, const Eigen::Vector3d &position, const rotation &orientation)
{
    for(Eigen::Index axis = 0; axis < 3; ++axis)
        CHECK(recorded(axis, 3) == Catch::Approx(position[axis]).margin(float_step));
    CHECK(recorded.topLeftCorner<3, 3>().isApprox(orientation, float_step));
}

}

TEST_CASE("a task space window built over an arm that has published nothing, or over no pose, refuses and names both ends", "[manipulator]")
{
    arm_publisher unheld;
    const std::shared_ptr<arm_publisher> published = publishing(chosen_snapshot());

    REQUIRE_THROWS_MATCHES(task_space_window("Task space", unheld.reader(), std::weak_ptr<owned_arm>(), reference, std::make_shared<edited_pose>()), std::invalid_argument,
                           Message("praxis: the task space window was given no published arm state to hold"));
    REQUIRE_THROWS_MATCHES(task_space_window("Task space", published->reader(), std::weak_ptr<owned_arm>(), reference, nullptr), std::invalid_argument,
                           Message("praxis: the task space window was given no shared pose to hold"));
}

TEST_CASE("a task space panel left with no publication draws one panel stating that", "[manipulator]")
{
    const std::shared_ptr<arm_publisher> published = publishing(chosen_snapshot());
    task_space_window panel("Task space", published->reader(), std::weak_ptr<owned_arm>(), reference, std::make_shared<edited_pose>());
    published->publish(nullptr);

    CHECK(geometry_of([&panel] { panel.render(); }) == stating_absence("Task space"));
}

TEST_CASE("a task space window draws sliders in preview and fields in simulation", "[manipulator][controls]")
{
    const std::shared_ptr<arm_publisher> published = publishing(chosen_snapshot());
    const std::shared_ptr<edited_pose> held        = holding(chosen_position, chosen_euler_degrees);
    task_space_window previewing("Task space", published->reader(), std::weak_ptr<owned_arm>(), reference, held, {shape::ptp, mode::preview});
    task_space_window simulating("Task space", published->reader(), std::weak_ptr<owned_arm>(), reference, held, {shape::lin, mode::simulation});

    const std::size_t previewed = geometry_of([&previewing] { previewing.render(); });
    const std::size_t simulated = geometry_of([&simulating] { simulating.render(); });

    CHECK(previewed == geometry_of([&held] { draw_slider_group(*held); }));
    CHECK(simulated == geometry_of([&held] { draw_field_group(*held); }));
    CHECK(previewed != simulated);
}

TEST_CASE("an offset entered at a task space window's position slider is previewed as the pose it composes", "[manipulator][controls]")
{
    resolved.clear();
    straight.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed               = composing(loop);
    const std::shared_ptr<edited_pose> held = std::make_shared<edited_pose>();
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    task_space_window panel("Task space", placed.seen, placed.owned, reference, held, {shape::ptp, mode::preview});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    enter_first_offset(frames, draw);
    static_cast<void>(loop.drain());

    REQUIRE(!resolved.empty());
    CHECK(straight.empty());
    reaches(resolved.back(), Eigen::Vector3d{slider_offset, 0.0, 0.0}, rotation::Identity());
}

TEST_CASE("moving from a task space window carries the shape the trajectory cycle stands on", "[manipulator][controls]")
{
    resolved.clear();
    straight.clear();

    const shape selected = GENERATE(shape::ptp, shape::lin);
    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed = composing(loop);
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    task_space_window panel("Task space", placed.seen, placed.owned, reference, holding(chosen_position, chosen_euler_degrees), {selected, mode::simulation});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    press_move(frames, draw);
    static_cast<void>(loop.drain());

    if(selected == shape::lin)
    {
        REQUIRE(!straight.empty());
        reaches(straight.back(), chosen_position, chosen_orientation);
        return;
    }

    REQUIRE(!resolved.empty());
    CHECK(straight.empty());
    reaches(resolved.front(), chosen_position, chosen_orientation);
}

TEST_CASE("two task space windows over one shared pose read one pose", "[manipulator][controls]")
{
    resolved.clear();
    straight.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed               = composing(loop);
    const std::shared_ptr<edited_pose> held = std::make_shared<edited_pose>();
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    task_space_window seeding("Task space##1", placed.seen, placed.owned, reference, held, {shape::ptp, mode::simulation});
    task_space_window moving("Task space##2", placed.seen, placed.owned, reference, held, {shape::ptp, mode::simulation});

    imgui_frame frames;
    start_navigating(frames, over_both(seeding, moving, "Task space##1"));
    press_reset(frames, over_both(seeding, moving, "Task space##1"));
    press_move(frames, over_both(seeding, moving, "Task space##2"));
    static_cast<void>(loop.drain());

    REQUIRE(!resolved.empty());
    reaches(resolved.front(), chosen_position, chosen_orientation);
    CHECK(moving.state().shape == shape::ptp);
    CHECK(moving.state().mode == mode::simulation);
}

TEST_CASE("a task space window in preview seeds the shared pose from the arm and previews from there", "[manipulator][controls]")
{
    resolved.clear();
    straight.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed               = composing(loop);
    const std::shared_ptr<edited_pose> held = std::make_shared<edited_pose>();
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    task_space_window panel("Task space", placed.seen, placed.owned, reference, held, {shape::ptp, mode::preview});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    press_reset(frames, draw);
    static_cast<void>(loop.drain());

    CHECK(held->position.cast<double>().isApprox(chosen_position, float_step));
    CHECK(held->euler_degrees.cast<double>().isApprox(chosen_euler_degrees, float_step));
    REQUIRE(!resolved.empty());
    reaches(resolved.back(), chosen_position, chosen_orientation);
}

TEST_CASE("a task space window in preview seeds nothing where the publication carries no tool pose", "[manipulator][controls]")
{
    resolved.clear();
    straight.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed               = composing(loop);
    const std::shared_ptr<edited_pose> held = std::make_shared<edited_pose>();
    placed.publishing->publish(std::make_shared<const arm_snapshot>(poseless_snapshot()));
    task_space_window panel("Task space", placed.seen, placed.owned, reference, held, {shape::ptp, mode::preview});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    press_reset(frames, draw);
    static_cast<void>(loop.drain());

    CHECK(held->position.cast<double>().isZero(float_step));
    CHECK(held->euler_degrees.cast<double>().isZero(float_step));
    CHECK(resolved.empty());
    CHECK(straight.empty());
}

TEST_CASE("a task space window in preview previews the pose the orientation order it was moved to composes", "[manipulator][controls]")
{
    resolved.clear();
    straight.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed               = composing(loop);
    const std::shared_ptr<edited_pose> held = holding(chosen_position, chosen_euler_degrees);
    placed.publishing->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    task_space_window panel("Task space", placed.seen, placed.owned, reference, held, {shape::ptp, mode::preview});

    imgui_frame frames;
    const drawing draw = over_alone(panel);
    start_navigating(frames, draw);
    take_first_euler_order(frames, draw);
    static_cast<void>(loop.drain());

    REQUIRE(!resolved.empty());
    CHECK(straight.empty());
    reaches(resolved.back(), chosen_position, reference.rotation_matrix_from_euler(chosen_euler_degrees * radians_per_degree, first_offered));
    CHECK(!resolved.back().topLeftCorner<3, 3>().isApprox(chosen_orientation, float_step));
}

TEST_CASE("a task space window moving point to point seeds nothing where the publication carries no tool pose", "[manipulator][controls]")
{
    resolved.clear();
    straight.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed               = composing(loop);
    const std::shared_ptr<edited_pose> held = std::make_shared<edited_pose>();
    placed.publishing->publish(std::make_shared<const arm_snapshot>(poseless_snapshot()));
    task_space_window panel("Task space", placed.seen, placed.owned, reference, held, {shape::ptp, mode::simulation});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    press_reset(frames, draw);
    static_cast<void>(loop.drain());

    CHECK(held->position.cast<double>().isZero(float_step));
    CHECK(held->euler_degrees.cast<double>().isZero(float_step));
    CHECK(resolved.empty());
    CHECK(straight.empty());
}
