#include "window_stage.h"

#include "praxis/manipulator/edited_pose.h"
#include "praxis/manipulator/tool_jog_window.h"
#include "praxis/manipulator/screw_jog_window.h"
#include "praxis/manipulator/task_space_window.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/axis_order.h"
#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_approx.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <Eigen/Core>

#include <memory>
#include <cstdint>
#include <utility>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

const rigid_motion::frame_ops reference = rigid_motion::baseline().frame;

// No two of them equal and none a right angle or a straight one, so a pose carried the wrong way
// through the seam lands where none of them is.
const Eigen::Vector3d chosen_euler_degrees{37.0, 52.0, -19.0};
const Eigen::Vector3d chosen_position{0.25, -0.4, 0.7};
const rotation chosen_orientation = reference.rotation_matrix_from_euler(chosen_euler_degrees * radians_per_degree, axis_order::zyx);

// Where the tool pose stands before the arm is handed its tool offset, sharing no component with any
// other position here.
const Eigen::Vector3d flange_position{0.1, -0.35, 0.55};

// A pose something other than a seed left held, apart from every published value.
const Eigen::Vector3d elsewhere_position{-0.3, 0.45, 0.15};
const Eigen::Vector3d elsewhere_euler_degrees{-61.0, 14.0, 83.0};

constexpr const char *typed_offset = "0.6";
constexpr double typed_value       = 0.6;

// The pose is held in single precision, so a component reaching the seam is the component behind it
// to within one float step of it.
constexpr double float_step = 1.0e-4;

enum class panel : std::uint8_t
{
    task_space,
    screw_jog,
    tool_jog
};

arm_snapshot chosen_snapshot()
{
    return at_rest(configuration(0.0, 0.0), chosen_position, chosen_orientation);
}

arm_snapshot poseless_snapshot()
{
    return at_rest(configuration(0.0, 0.0), praxis::unexpected(refusal::no_solution), praxis::unexpected(refusal::no_solution));
}

arm_snapshot before_offset_snapshot(const Eigen::Vector3d &position = flange_position)
{
    arm_snapshot seen      = at_rest(configuration(0.0, 0.0), position, chosen_orientation);
    seen.tool_offset_known = false;

    return seen;
}

std::unique_ptr<scene::imgui_window> opened(panel kind, arm_reader seen, std::shared_ptr<edited_pose> edited, const char *name, control_mode chosen = control_mode::preview)
{
    switch(kind)
    {
        case panel::task_space:
            return std::make_unique<task_space_window>(name, seen, std::weak_ptr<owned_arm>(), reference, std::move(edited),
                                                       task_space_window::settings{task_space_window::motion_shape::ptp, chosen});
        case panel::screw_jog:
            return std::make_unique<screw_jog_window>(name, seen, std::weak_ptr<owned_arm>(), reference, std::move(edited), screw_jog_window::settings{chosen});
        case panel::tool_jog:
            return std::make_unique<tool_jog_window>(name, seen, std::weak_ptr<owned_arm>(), reference, std::move(edited), tool_jog_window::settings{chosen});
    }

    return nullptr;
}

// Every panel draws its first pose control one row under its mode cycle, and a row is entered at its
// leftmost, which is the first component of the position.
void enter_first_pose_control(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_Home);
    tap(frames, draw, ImGuiKey_DownArrow);
    type_at_cursor(frames, draw, typed_offset);
}

void stands_at(const edited_pose &edited, const Eigen::Vector3d &position, const Eigen::Vector3d &euler_degrees, pose_standing standing)
{
    CHECK(edited.position.cast<double>().isApprox(position, float_step));
    CHECK(edited.euler_degrees.cast<double>().isApprox(euler_degrees, float_step));
    CHECK(edited.standing == standing);
}

}

TEST_CASE("a window over a shared pose nothing has set shows the published tool pose from its first frame and marks it seeded", "[manipulator][controls]")
{
    const panel kind                               = GENERATE(panel::task_space, panel::screw_jog, panel::tool_jog);
    const control_mode chosen                      = GENERATE(control_mode::preview, control_mode::simulation);
    const std::shared_ptr<arm_publisher> published = publishing(chosen_snapshot());
    const auto shared                              = std::make_shared<edited_pose>();
    const auto window                              = opened(kind, published->reader(), shared, "Shared pose", chosen);

    imgui_frame frames;
    frames.draw(over(*window));

    stands_at(*shared, chosen_position, chosen_euler_degrees, pose_standing::seeded);
}

TEST_CASE("a window over a publication carrying no tool pose seeds the shared pose from the one that follows", "[manipulator][controls]")
{
    const panel kind                               = GENERATE(panel::task_space, panel::screw_jog, panel::tool_jog);
    const std::shared_ptr<arm_publisher> published = publishing(poseless_snapshot());
    const auto shared                              = std::make_shared<edited_pose>();
    const auto window                              = opened(kind, published->reader(), shared, "Shared pose");

    imgui_frame frames;
    frames.draw(over(*window));
    REQUIRE(shared->position.cast<double>().isZero(float_step));
    REQUIRE(shared->standing == pose_standing::unset);

    published->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    frames.draw(over(*window));

    stands_at(*shared, chosen_position, chosen_euler_degrees, pose_standing::seeded);
}

TEST_CASE("a window seeds the shared pose provisionally before the tool offset is known and once more when it is", "[manipulator][controls]")
{
    const panel kind                               = GENERATE(panel::task_space, panel::screw_jog, panel::tool_jog);
    const std::shared_ptr<arm_publisher> published = publishing(before_offset_snapshot());
    const auto shared                              = std::make_shared<edited_pose>();
    const auto window                              = opened(kind, published->reader(), shared, "Shared pose");

    imgui_frame frames;
    frames.draw(over(*window));
    stands_at(*shared, flange_position, chosen_euler_degrees, pose_standing::provisional);

    published->publish(std::make_shared<const arm_snapshot>(before_offset_snapshot(elsewhere_position)));
    frames.draw(over(*window));
    stands_at(*shared, flange_position, chosen_euler_degrees, pose_standing::provisional);

    published->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    frames.draw(over(*window));
    stands_at(*shared, chosen_position, chosen_euler_degrees, pose_standing::seeded);
}

TEST_CASE("a shared pose entered at a window before any tool pose was published is not overwritten by the seed", "[manipulator][controls]")
{
    const panel kind                               = GENERATE(panel::task_space, panel::screw_jog, panel::tool_jog);
    const std::shared_ptr<arm_publisher> published = publishing(poseless_snapshot());
    const auto shared                              = std::make_shared<edited_pose>();
    const auto window                              = opened(kind, published->reader(), shared, "Shared pose");

    imgui_frame frames;
    const drawing draw = over(*window);
    start_navigating(frames, draw);
    enter_first_pose_control(frames, draw);
    published->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    frames.draw(draw);

    stands_at(*shared, Eigen::Vector3d{typed_value, 0.0, 0.0}, Eigen::Vector3d::Zero(), pose_standing::held);
}

TEST_CASE("a window over a shared pose already held leaves it as it stands", "[manipulator][controls]")
{
    const panel kind                               = GENERATE(panel::task_space, panel::screw_jog, panel::tool_jog);
    const std::shared_ptr<arm_publisher> published = publishing(chosen_snapshot());
    const auto shared                              = std::make_shared<edited_pose>();
    shared->position                               = elsewhere_position.cast<float>();
    shared->euler_degrees                          = elsewhere_euler_degrees.cast<float>();
    shared->standing                               = pose_standing::held;
    const auto window                              = opened(kind, published->reader(), shared, "Shared pose");

    imgui_frame frames;
    frames.draw(over(*window));
    frames.draw(over(*window));

    stands_at(*shared, elsewhere_position, elsewhere_euler_degrees, pose_standing::held);
}

TEST_CASE("a window over a shared pose already seeded leaves it as it stands", "[manipulator][controls]")
{
    const panel kind                               = GENERATE(panel::task_space, panel::screw_jog, panel::tool_jog);
    const std::shared_ptr<arm_publisher> published = publishing(chosen_snapshot());
    const auto shared                              = std::make_shared<edited_pose>();
    shared->position                               = elsewhere_position.cast<float>();
    shared->euler_degrees                          = elsewhere_euler_degrees.cast<float>();
    shared->standing                               = pose_standing::seeded;
    const auto window                              = opened(kind, published->reader(), shared, "Shared pose");

    imgui_frame frames;
    frames.draw(over(*window));
    frames.draw(over(*window));

    stands_at(*shared, elsewhere_position, elsewhere_euler_degrees, pose_standing::seeded);
}

TEST_CASE("a shared pose entered at one window is left standing by every window drawn over it later", "[manipulator][controls]")
{
    const bool before_offset                       = GENERATE(false, true);
    const panel later                              = GENERATE(panel::task_space, panel::screw_jog, panel::tool_jog);
    const std::shared_ptr<arm_publisher> published = publishing(before_offset ? before_offset_snapshot() : chosen_snapshot());
    const auto shared                              = std::make_shared<edited_pose>();
    const auto first                               = opened(panel::task_space, published->reader(), shared, "Entered");

    imgui_frame frames;
    const drawing draw = over(*first);
    start_navigating(frames, draw);
    enter_first_pose_control(frames, draw);
    const Eigen::Vector3d seeded = before_offset ? flange_position : chosen_position;
    const Eigen::Vector3d entered{typed_value, seeded[1], seeded[2]};
    REQUIRE(shared->position.cast<double>().isApprox(entered, float_step));

    published->publish(std::make_shared<const arm_snapshot>(chosen_snapshot()));
    const auto second = opened(later, published->reader(), shared, "Drawn later");
    frames.draw(
            [&first, &second]
            {
                first->render();
                second->render();
            });

    stands_at(*shared, entered, chosen_euler_degrees, pose_standing::held);
}
