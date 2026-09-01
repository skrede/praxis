#include "window_stage.h"

#include "praxis/manipulator/option_widgets.h"
#include "praxis/manipulator/joint_control_window.h"

#include "praxis/trajectory/trajectory.h"

#include "praxis/rigid_motion/angles.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>

#include <Eigen/Core>

#include <imgui.h>

#include <span>
#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;
using Catch::Matchers::Message;

namespace {

// Neither of them a whole number of degrees and neither the other's negation, so a field seeded in
// radians formats differently from one seeded in degrees.
const joint_vector chosen_joints = configuration(0.3, -0.7);

enum class widget_choice : std::uint8_t
{
    sliders,
    fields
};

// Three joints where the window was built over two, so a panel drawn from a list the seeding never
// re-derived carries a label the case did not write.
const joint_vector republished_joints = []
{
    joint_vector q(3);
    q << 0.3, -0.7, 1.1;

    return q;
}();

// Neither a whole number of radians nor one whose conversion is its own inverse, so a field seeded
// through the wrong direction lands where the case's own value is not.
constexpr const char *typed_joint         = "15";
constexpr double typed_joint_degrees      = 15.0;
const joint_vector standing_configuration = configuration(0.9, -0.9);

std::vector<joint_vector> requested;

// The waypoints a move hands the factory, taken without a motion having to play out: the factory
// answers a refusal, which commands no motion and is not one of the refusals that unload.
expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> recorded_waypoints(std::span<const trajectory::configuration> waypoints, const trajectory::configuration &,
                                                                                        const trajectory::configuration_limits &)
{
    if(!waypoints.empty())
        requested.push_back(waypoints.front());

    return praxis::unexpected(refusal::no_solution);
}

composed_arm composing(praxis::scheduler::scheduler &loop)
{
    const strand work    = *loop.make_strand();
    const auto driven    = std::make_shared<scene_robot>(two_joint_arm(robot_ops{}));
    const auto published = std::make_shared<arm_publisher>();
    const auto control   = std::make_shared<robot_controller>(*driven, motion_ops{}, trajectory::path_ops{}, task_trajectory_ops{}, trajectory::time_scaling_ops{},
                                                              trajectory::trajectory_ops{.joint_space_waypoints = &recorded_waypoints}, rigid_motion::screw_ops{});

    return composed_arm{published->reader(), published, std::make_shared<owned_arm>(work, work, driven, control, published)};
}

// The arm's own configuration, read on the strand that owns it rather than off the publication.
joint_vector held_by(const composed_arm &placed, praxis::scheduler::scheduler &loop)
{
    const auto read = std::make_shared<joint_vector>();
    command(placed.owned, [read](robot_controller &control, scene_robot &) { *read = control.joint_positions(); });
    static_cast<void>(loop.drain());

    return *read;
}

arm_snapshot upright(const joint_vector &joints)
{
    return at_rest(joints, Eigen::Vector3d::Zero(), rotation::Identity());
}

// The panel a joint window draws in the mode it is given, written out here rather than read off the
// window: the labels, the seeded values and which controls the mode puts on the last row are the
// case's own claim about what the window put on screen.
void draw_reference(std::span<const char *const> labels, const Eigen::VectorXf &shown, control_mode drawn = control_mode::simulation)
{
    option_cycle<control_mode, 2> mode(drawn, {control_mode::preview, control_mode::simulation}, {"Preview", "Simulation"});
    option_cycle<widget_choice, 2> widgets(widget_choice::fields, {widget_choice::sliders, widget_choice::fields}, {"Sliders", "Fields"});
    Eigen::VectorXf held = shown;

    ImGui::Begin("Joint control");
    render_option_cycle("Control mode", mode);
    render_option_cycle("Widgets", widgets);
    for(Eigen::Index field = 0; field < held.size(); ++field)
        ImGui::InputFloat(labels[static_cast<std::size_t>(field)], &held[field], 1.f, 10.f);
    static_cast<void>(ImGui::Button("Reset to zero"));
    static_cast<void>(ImGui::Button("Reset to current"));
    if(drawn == control_mode::simulation)
    {
        ImGui::SameLine();
        static_cast<void>(ImGui::Button("Move"));
    }
    ImGui::End();
}

// The first joint control stands two rows under the pane's first, which the two cycles occupy.
void enter_first_joint(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_Home);
    tap(frames, draw, ImGuiKey_DownArrow);
    tap(frames, draw, ImGuiKey_DownArrow);
    type_at_cursor(frames, draw, typed_joint);
}

// The pane's last row carries the seeding control and the move control side by side, and a row is
// entered at its leftmost.
void press_seed(imgui_frame &frames, const drawing &draw)
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

// A pane left to size itself keeps the extent the frame it first appeared on gave it, so one that
// gained a row later is drawn against a stale extent while one built over the later publication is
// not. An extent named ahead of the frame takes that difference out of the comparison.
const ImVec2 pane_extent{600.f, 400.f};

drawing at_fixed_extent(const drawing &draw)
{
    return [draw]
    {
        ImGui::SetNextWindowSize(pane_extent);
        draw();
    };
}

// The keyboard cursor a seeding press leaves standing on the pane's last row is geometry like every
// other vertex, so the panel and the reference it is read against are driven through one sequence.
// The context a frame fixture owns is global, which is why only one of them is open at a time.
std::size_t seeded_over(const drawing &draw)
{
    const drawing sized = at_fixed_extent(draw);

    imgui_frame frames;
    start_navigating(frames, sized);
    press_seed(frames, sized);

    return frames.signature();
}

std::size_t seeded_after(const arm_snapshot &later)
{
    const std::shared_ptr<arm_publisher> published = publishing(upright(chosen_joints));
    joint_control_window panel("Joint control", published->reader(), std::weak_ptr<owned_arm>());
    published->publish(std::make_shared<const arm_snapshot>(later));

    return seeded_over(over(panel));
}

// One drawing at the named extent and no input at all, which is what a re-derivation reached by the
// publication rather than by a control has to answer for.
std::size_t drawn_over(const drawing &draw)
{
    return geometry_of(at_fixed_extent(draw));
}

// A field moved and the seeding control then pressed. Both steps leave the keyboard cursor on the
// pane's last row, so a panel driven through this and a reference driven through the press alone
// stand at the same vertex once the fields agree.
std::size_t typed_then_seeded(const drawing &draw)
{
    const drawing sized = at_fixed_extent(draw);

    imgui_frame frames;
    start_navigating(frames, sized);
    enter_first_joint(frames, sized);
    press_seed(frames, sized);

    return frames.signature();
}

}

TEST_CASE("a joint control window built over an arm that has published nothing refuses and names both ends", "[manipulator]")
{
    arm_publisher unheld;

    REQUIRE_THROWS_MATCHES(joint_control_window("Joint control", unheld.reader(), std::weak_ptr<owned_arm>()), std::invalid_argument,
                           Message("praxis: the joint control window was given no published arm state to hold"));
}

// A publication withdrawn under a live window is the route to the render path's own answer: a window
// is refused where it is built, so the reader it keeps has published by the time it can be drawn.
TEST_CASE("a joint control panel left with no publication draws one panel stating that", "[manipulator]")
{
    const std::shared_ptr<arm_publisher> published = publishing(at_rest(chosen_joints, Eigen::Vector3d::Zero(), rotation::Identity()));
    joint_control_window panel("Joint control", published->reader(), std::weak_ptr<owned_arm>());
    published->publish(nullptr);

    CHECK(geometry_of([&panel] { panel.render(); }) == stating_absence("Joint control"));
}

TEST_CASE("a joint control window seeds its fields from the published joints in degrees and labels them in order", "[manipulator][controls]")
{
    const std::shared_ptr<arm_publisher> published = publishing(at_rest(chosen_joints, Eigen::Vector3d::Zero(), rotation::Identity()));
    joint_control_window panel("Joint control", published->reader(), std::weak_ptr<owned_arm>());

    const Eigen::Vector2f in_degrees = (chosen_joints * degrees_per_radian).cast<float>();
    const Eigen::Vector2f in_radians = chosen_joints.cast<float>();
    const char *const named[2]{"j1", "j2"};
    const char *const misnamed[2]{"a1", "a2"};

    const std::size_t drawn = geometry_of([&panel] { panel.render(); });

    CHECK(drawn == geometry_of([&in_degrees, &named] { draw_reference(named, in_degrees); }));
    CHECK(drawn != geometry_of([&in_radians, &named] { draw_reference(named, in_radians); }));
    CHECK(drawn != geometry_of([&in_degrees, &misnamed] { draw_reference(misnamed, in_degrees); }));
}

TEST_CASE("a joint control window answers the mode it was built with and offers no document without a key path", "[manipulator]")
{
    const std::shared_ptr<arm_publisher> published = publishing(at_rest(chosen_joints, Eigen::Vector3d::Zero(), rotation::Identity()));
    joint_control_window defaulted("Joint control", published->reader(), std::weak_ptr<owned_arm>());
    joint_control_window previewing("Joint control", published->reader(), std::weak_ptr<owned_arm>(), joint_control_window::settings{control_mode::preview});
    joint_control_window addressed("Joint control", published->reader(), std::weak_ptr<owned_arm>(), joint_control_window::settings{control_mode::simulation}, "joint_control");

    CHECK(defaulted.state().mode == control_mode::simulation);
    CHECK(previewing.state().mode == control_mode::preview);
    CHECK(defaulted.as_configurable() == nullptr);
    CHECK(addressed.as_configurable() == &addressed);
    CHECK(addressed.settings_path() == "joint_control");
}

TEST_CASE("a joint control window in preview drives the arm from the field an operator typed", "[manipulator][controls]")
{
    requested.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed = composing(loop);
    placed.publishing->publish(std::make_shared<const arm_snapshot>(upright(configuration(0.0, 0.0))));
    joint_control_window panel("Joint control", placed.seen, placed.owned, joint_control_window::settings{control_mode::preview});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    enter_first_joint(frames, draw);
    static_cast<void>(loop.drain());

    CHECK(is_approx_equal(held_by(placed, loop), configuration(to_radians(typed_joint_degrees), 0.0), round_trip));
    CHECK(requested.empty());
}

TEST_CASE("a joint control window in preview issues nothing on a frame no widget moved", "[manipulator][controls]")
{
    requested.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed = composing(loop);
    placed.publishing->publish(std::make_shared<const arm_snapshot>(upright(configuration(0.0, 0.0))));
    joint_control_window panel("Joint control", placed.seen, placed.owned, joint_control_window::settings{control_mode::preview});

    command(placed.owned, [](robot_controller &, scene_robot &driven) { driven.set_joint_positions(standing_configuration); });
    static_cast<void>(loop.drain());

    imgui_frame frames;
    const drawing draw = over(panel);
    frames.draw(draw);
    frames.draw(draw);
    static_cast<void>(loop.drain());

    CHECK(is_approx_equal(held_by(placed, loop), standing_configuration, round_trip));
}

TEST_CASE("a joint control window's move carries the configuration its fields hold", "[manipulator][controls]")
{
    requested.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed = composing(loop);
    placed.publishing->publish(std::make_shared<const arm_snapshot>(upright(configuration(0.0, 0.0))));
    joint_control_window panel("Joint control", placed.seen, placed.owned, joint_control_window::settings{control_mode::simulation});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    enter_first_joint(frames, draw);
    press_move(frames, draw);
    static_cast<void>(loop.drain());

    REQUIRE(requested.size() == 1u);
    CHECK(is_approx_equal(requested.front(), configuration(to_radians(typed_joint_degrees), 0.0), round_trip));
}

TEST_CASE("a joint control window's seeding control re-labels when the publication's joint count changed", "[manipulator][controls]")
{
    const Eigen::VectorXf in_degrees = (republished_joints * degrees_per_radian).cast<float>();
    const char *const named[3]{"j1", "j2", "j3"};

    CHECK(seeded_after(upright(republished_joints)) == seeded_over([&in_degrees, &named] { draw_reference(named, in_degrees); }));
}

TEST_CASE("a joint control window in preview re-derives its fields where the publication's joint count changed", "[manipulator][controls]")
{
    const std::shared_ptr<arm_publisher> published = publishing(upright(chosen_joints));
    joint_control_window panel("Joint control", published->reader(), std::weak_ptr<owned_arm>(), joint_control_window::settings{control_mode::preview});
    published->publish(std::make_shared<const arm_snapshot>(upright(republished_joints)));

    const Eigen::VectorXf in_degrees = (republished_joints * degrees_per_radian).cast<float>();
    const char *const named[3]{"j1", "j2", "j3"};

    CHECK(drawn_over(over(panel)) == drawn_over([&in_degrees, &named] { draw_reference(named, in_degrees, control_mode::preview); }));
}

TEST_CASE("a joint control window in preview returns its fields to the publication through its seeding control", "[manipulator][controls]")
{
    const std::shared_ptr<arm_publisher> published = publishing(upright(chosen_joints));
    joint_control_window panel("Joint control", published->reader(), std::weak_ptr<owned_arm>(), joint_control_window::settings{control_mode::preview});

    const Eigen::VectorXf in_degrees = (chosen_joints * degrees_per_radian).cast<float>();
    const char *const named[2]{"j1", "j2"};

    CHECK(typed_then_seeded(over(panel)) == seeded_over([&in_degrees, &named] { draw_reference(named, in_degrees, control_mode::preview); }));
}
