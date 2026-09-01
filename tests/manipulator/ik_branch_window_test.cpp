#include "window_stage.h"
#include "drawn_chain.h"

#include "praxis/manipulator/ik_branch_window.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>

#include <threepp/scenes/Scene.hpp>

#include <imgui.h>

#include <Eigen/Core>

#include <memory>
#include <string>
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

constexpr const char *panel_title = "Solutions";

const rigid_motion::frame_ops reference = rigid_motion::baseline().frame;

const Eigen::Vector3d origin(Eigen::Vector3d::Zero());
const rotation upright(rotation::Identity());

// Three postures a two-joint arm reaches one target from, with the one the arm stands at second, so
// a list that opened on its first entry or on the last would read as the wrong one.
std::vector<joint_vector> three_postures()
{
    return {configuration(1.0, 0.0), configuration(0.0, 0.0), configuration(2.0, 0.0)};
}

arm_snapshot standing_at(const joint_vector &joints, const std::vector<joint_vector> &solutions)
{
    arm_snapshot seen = at_rest(joints, origin, upright);
    seen.solutions    = solutions;

    return seen;
}

praxis::screw_axis revolute_screw(const Eigen::Vector3d &through)
{
    return rigid_motion::baseline().screw.screw_axis_from_point_direction_pitch(through, Eigen::Vector3d::UnitZ(), 0.0).value();
}

std::vector<praxis::screw_axis> two_axes()
{
    return {revolute_screw(Eigen::Vector3d::Zero()), revolute_screw(Eigen::Vector3d(static_cast<double>(link_length), 0.0, 0.0))};
}

// What one ask reached and what one pick commanded, kept beside the stage so a case reads the two
// apart: the route is called where the ask is made, the command runs on the arm's strand.
struct asked_of
{
    std::size_t routed = 0;
    std::size_t ran    = 0;
    std::vector<joint_vector> commanded;
    transform target = transform::Identity();
};

// A scene needs no graphics context and a renderer robot needs no display, so the whole stage is
// built headlessly.
struct stage
{
    stage()
            : loop(inline_workers, clock_source{&reading})
            , scene(threepp::Scene::create())
            , published(std::make_shared<arm_publisher>())
            , held(std::make_shared<edited_pose>())
            , shown(two_joint_handle(), attached_models{}, *scene, loop.main_strand(), published->reader(), rigid_motion::baseline().screw, rigid_motion::screw_slot_set{})
    {
        publish(standing_at(configuration(0.0, 0.0), {}));
        REQUIRE(shown.initialize().has_value());
        REQUIRE(shown.set_joint_screws(transform::Identity(), two_axes()).has_value());
    }

    void publish(const arm_snapshot &seen)
    {
        published->publish(std::make_shared<const arm_snapshot>(seen));
    }

    std::shared_ptr<owned_arm> arm()
    {
        const strand work    = *loop.make_strand();
        const auto driven    = std::make_shared<scene_robot>(two_joint_arm(robot_ops{}));
        const auto commanded = std::make_shared<robot_controller>(*driven, composing_motion(), composing_path(), task_trajectory_ops{}, composing_time_scaling(),
                                                                  trajectory::trajectory_ops{}, rigid_motion::baseline().screw);
        owned                = std::make_shared<owned_arm>(work, work, driven, commanded, published);

        return owned;
    }

    threepp::Object3D *figure(std::size_t at)
    {
        return chain_node(*scene, loadable_robot_stencil::solution_figure_name(at));
    }

    praxis::scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<arm_publisher> published;
    std::shared_ptr<edited_pose> held;
    std::shared_ptr<owned_arm> owned;
    loadable_robot_stencil shown;
};

// A route that says what it was asked and what the command it answered did, and nothing else: no
// solver is named here, which is the composition's part and not the window's.
ik_branch_window::solve_route counting(asked_of &into)
{
    return [&into](const transform &target)
    {
        into.routed += 1u;
        into.target = target;

        return ik_branch_window::solve_command([&into](robot_controller &) { into.ran += 1u; });
    };
}

// A route the composition wired to nothing, which is a slot no binding was given.
ik_branch_window::solve_route refusing(asked_of &into)
{
    return [&into](const transform &)
    {
        into.routed += 1u;

        return ik_branch_window::solve_command();
    };
}

drawing over_alone(ik_branch_window &panel)
{
    return [&panel] { panel.render(); };
}

void press_solve(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_Space);
}

// The list stands one row above the panel's last, and offers its entries in a popup of its own.
void take_entry(imgui_frame &frames, const drawing &draw, std::size_t below)
{
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_UpArrow);
    tap(frames, draw, ImGuiKey_Space);
    for(std::size_t step = 0; step < below; ++step)
        tap(frames, draw, ImGuiKey_DownArrow);
    tap(frames, draw, ImGuiKey_Space);
}

}

TEST_CASE("a branch list window built over an arm that has published nothing, or over no pose, refuses and names both ends", "[manipulator][branches]")
{
    stage headless;
    arm_publisher unheld;

    REQUIRE_THROWS_MATCHES(ik_branch_window(panel_title, unheld.reader(), std::weak_ptr<owned_arm>(), reference, headless.held, headless.shown, ik_branch_window::solve_route()),
                           std::invalid_argument, Message("praxis: the branch list window was given no published arm state to hold"));
    REQUIRE_THROWS_MATCHES(ik_branch_window(panel_title, headless.published->reader(), std::weak_ptr<owned_arm>(), reference, nullptr, headless.shown, ik_branch_window::solve_route()),
                           std::invalid_argument, Message("praxis: the branch list window was given no shared pose to hold"));
}

TEST_CASE("asking for a solve runs one command on the arm's strand and the window makes no solve of its own", "[manipulator][branches]")
{
    stage headless;
    asked_of seen;
    ik_branch_window panel(panel_title, headless.published->reader(), headless.arm(), reference, headless.held, headless.shown, counting(seen));

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    press_solve(frames, draw);
    static_cast<void>(headless.loop.drain());

    CHECK(seen.routed == 1u);
    CHECK(seen.ran == 1u);
}

TEST_CASE("moving the target pose or drawing the panel again sends no command at all", "[manipulator][branches]")
{
    stage headless;
    asked_of seen;
    ik_branch_window panel(panel_title, headless.published->reader(), headless.arm(), reference, headless.held, headless.shown, counting(seen));

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    headless.held->position = Eigen::Vector3f(0.2f, -0.3f, 0.4f);
    frames.draw(draw);
    headless.held->euler_degrees = Eigen::Vector3f(15.f, 25.f, -35.f);
    frames.draw(draw);
    static_cast<void>(headless.loop.drain());

    CHECK(seen.routed == 0u);
    CHECK(seen.ran == 0u);
}

TEST_CASE("the pose an ask carries is the pose the shared target composes", "[manipulator][branches]")
{
    stage headless;
    asked_of seen;
    headless.held->position = Eigen::Vector3f(0.2f, -0.3f, 0.4f);
    ik_branch_window panel(panel_title, headless.published->reader(), headless.arm(), reference, headless.held, headless.shown, counting(seen));

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    press_solve(frames, draw);
    static_cast<void>(headless.loop.drain());

    REQUIRE(seen.routed == 1u);
    CHECK(seen.target.isApprox(pose_matrix(*headless.held, reference)));
}

TEST_CASE("after a solve the list names one entry per distinct configuration and opens on the one the arm stands at", "[manipulator][branches]")
{
    stage headless;
    asked_of seen;
    ik_branch_window panel(panel_title, headless.published->reader(), headless.arm(), reference, headless.held, headless.shown, counting(seen));

    headless.publish(standing_at(configuration(0.0, 0.0), three_postures()));
    imgui_frame frames;
    frames.draw(over(panel));

    REQUIRE(panel.selected().has_value());
    CHECK(*panel.selected() == 1u);
}

TEST_CASE("the figures told are every distinct configuration but the one the arm stands at", "[manipulator][branches]")
{
    stage headless;
    asked_of seen;
    ik_branch_window panel(panel_title, headless.published->reader(), headless.arm(), reference, headless.held, headless.shown, counting(seen));

    headless.publish(standing_at(configuration(0.0, 0.0), three_postures()));
    imgui_frame frames;
    frames.draw(over(panel));
    headless.scene->updateMatrixWorld(true);

    CHECK(headless.figure(0) != nullptr);
    CHECK(headless.figure(1) != nullptr);
    CHECK(headless.figure(2) == nullptr);
}

TEST_CASE("picking another entry moves the arm to that configuration and leaves the list as it stood", "[manipulator][branches]")
{
    stage headless;
    asked_of seen;
    ik_branch_window panel(panel_title, headless.published->reader(), headless.arm(), reference, headless.held, headless.shown, counting(seen),
                           ik_branch_window::settings{control_mode::preview, true});

    headless.publish(standing_at(configuration(0.0, 0.0), three_postures()));
    imgui_frame frames;
    const drawing draw = over_alone(panel);
    start_navigating(frames, draw);
    take_entry(frames, draw, 2u);

    REQUIRE(panel.selected().has_value());
    CHECK(*panel.selected() == 2u);
    CHECK(seen.routed == 0u);

    static_cast<void>(headless.loop.drain());
    const std::shared_ptr<const arm_snapshot> after = headless.published->reader().read();

    REQUIRE(after != nullptr);
    CHECK(after->joints.isApprox(configuration(2.0, 0.0)));
}

TEST_CASE("a publication carrying no configuration leaves the list empty and says so in place of a list", "[manipulator][branches]")
{
    stage headless;
    asked_of seen;
    ik_branch_window panel(panel_title, headless.published->reader(), headless.arm(), reference, headless.held, headless.shown, counting(seen));

    imgui_frame frames;
    frames.draw(over(panel));

    CHECK_FALSE(panel.selected().has_value());
    CHECK(headless.figure(0) == nullptr);
}

TEST_CASE("a slot the composition wired to nothing leaves the window open and the arm where it was", "[manipulator][branches]")
{
    stage headless;
    asked_of seen;
    ik_branch_window panel(panel_title, headless.published->reader(), headless.arm(), reference, headless.held, headless.shown, refusing(seen));

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    press_solve(frames, draw);

    static_cast<void>(headless.loop.drain());
    const std::shared_ptr<const arm_snapshot> after = headless.published->reader().read();

    CHECK(seen.routed == 1u);
    CHECK(seen.ran == 0u);
    REQUIRE(after != nullptr);
    CHECK(after->joints.isApprox(configuration(0.0, 0.0)));
    CHECK_FALSE(panel.selected().has_value());
}

TEST_CASE("one window serves either slot, because which one is asked is carried in the route", "[manipulator][branches]")
{
    stage headless;
    asked_of numerical;
    asked_of analytic;
    ik_branch_window over_starts(panel_title, headless.published->reader(), headless.arm(), reference, headless.held, headless.shown, counting(numerical));
    ik_branch_window in_one_go(panel_title, headless.published->reader(), headless.arm(), reference, headless.held, headless.shown, counting(analytic));

    imgui_frame frames;
    const drawing draw = over(in_one_go);
    start_navigating(frames, draw);
    press_solve(frames, draw);
    static_cast<void>(headless.loop.drain());

    CHECK(numerical.routed == 0u);
    CHECK(analytic.routed == 1u);
    CHECK(analytic.ran == 1u);
}

TEST_CASE("a branch list window no key path was named for offers nothing", "[manipulator][branches]")
{
    stage headless;
    asked_of seen;
    ik_branch_window unnamed(panel_title, headless.published->reader(), headless.arm(), reference, headless.held, headless.shown, counting(seen));
    ik_branch_window named(panel_title, headless.published->reader(), headless.arm(), reference, headless.held, headless.shown, counting(seen), ik_branch_window::settings{},
                           "machine/ik_branch");

    CHECK(unnamed.as_configurable() == nullptr);
    CHECK(named.as_configurable() == &named);
    CHECK(named.settings_path() == "machine/ik_branch");
}
