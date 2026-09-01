#include "window_stage.h"

#include "praxis/manipulator/tool_jog_window.h"
#include "praxis/manipulator/task_space_window.h"

#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/axis_order.h"
#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <imgui.h>

#include <memory>
#include <vector>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

const rigid_motion::frame_ops reference_frame = rigid_motion::baseline().frame;

using mode  = control_mode;
using shape = task_space_window::motion_shape;

// The window extracts and rebuilds under this order unless its selector is driven elsewhere, so the
// published orientation is composed under it too.
constexpr axis_order window_order = axis_order::zyx;

// No two of them equal and none of them a right angle or a straight one, and the first inside the
// half turn the extraction confines a first angle to, so the published orientation is the triple
// the extraction hands back and all three faults land clear of it.
const Eigen::Vector3d published_euler_degrees{37.0, 52.0, -19.0};
const rotation published_orientation = reference_frame.rotation_matrix_from_euler(published_euler_degrees * radians_per_degree, window_order);

// Neither a right angle nor a straight one, for the same reason, and no component of the published
// triple, so a row carrying it cannot be mistaken for a row carrying the pose.
constexpr const char *typed_jog_degrees = "23";

const Eigen::Vector3d jog_euler_degrees{0.0, 0.0, 23.0};

// The window holds its angles in single precision, so a value reaching the seam is the degrees
// behind it to within one float step, which is 6.7e-8 radians at the widest of these angles.
constexpr double conversion_tolerance = 1.0e-7;

std::vector<rotation> extracted;
std::vector<Eigen::Vector3d> built_radians;

Eigen::Vector3d recorded_extraction(const rotation &r, axis_order order)
{
    extracted.push_back(r);

    return reference_frame.euler_from_rotation_matrix(r, order);
}

rotation recorded_build(const Eigen::Vector3d &euler_radians, axis_order order)
{
    built_radians.push_back(euler_radians);

    return reference_frame.rotation_matrix_from_euler(euler_radians, order);
}

rigid_motion::frame_ops framing()
{
    rigid_motion::frame_ops ops    = reference_frame;
    ops.euler_from_rotation_matrix = &recorded_extraction;
    ops.rotation_matrix_from_euler = &recorded_build;

    return ops;
}

// A jog is answered with the configuration it started from: the case reads the angles arriving at
// the seam, and an unbound slot would report a refusal on every frame the drive types into.
expected<joint_vector, refusal> unmoved(const kinematics &, const transform &, const Eigen::Vector3d &, const rotation &, const joint_vector &j0)
{
    return j0;
}

motion_ops jogging()
{
    return motion_ops{.task_space_pose = &position_as_configuration, .tool_frame_displace = &unmoved};
}

// The seeding path does nothing unless both the orientation and the position are published.
std::shared_ptr<const arm_snapshot> chosen_pose()
{
    return std::make_shared<const arm_snapshot>(at_rest(configuration(0.0, 0.0), Eigen::Vector3d::Constant(0.25), published_orientation));
}

// Componentwise and against the constant, never through the helper the site under test calls: an
// assertion routed through that helper cancels every perturbation of its use.
void reaches(const Eigen::Vector3d &recorded_radians, const Eigen::Vector3d &degrees)
{
    for(Eigen::Index angle = 0; angle < 3; ++angle)
        CHECK(recorded_radians[angle] == Catch::Approx(degrees[angle] * radians_per_degree).margin(conversion_tolerance));
}

// The point-to-point pane's last row carries the seeding control and the move control side by side,
// and a row is entered at its leftmost, so a sideways step reaches the second of the two.
void press_task_space_reset(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_Space);
}

void press_task_space_move(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_Space);
}

// The tool frame pane's seeding control is its fifth row, under the mode selector, the two pose
// fields and the order selector, and the jog orientation's last component is the pane's last row.
void press_tool_frame_reset(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_Home);
    for(int row = 0; row < 4; ++row)
        tap(frames, draw, ImGuiKey_DownArrow);
    tap(frames, draw, ImGuiKey_Space);
}

void enter_last_jog_angle(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    type_at_cursor(frames, draw, typed_jog_degrees);
}

}

TEST_CASE("the orientation a task space window builds a target pose from reaches the frame seam as the same orientation in radians", "[manipulator][controls]")
{
    extracted.clear();
    built_radians.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed = compose(loop, jogging(), rigid_motion::baseline().screw);
    placed.publishing->publish(chosen_pose());
    task_space_window panel("Task space", placed.seen, placed.owned, framing(), std::make_shared<edited_pose>(), {shape::ptp, mode::simulation});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    press_task_space_reset(frames, draw);
    press_task_space_move(frames, draw);
    static_cast<void>(loop.drain());

    REQUIRE(extracted.size() == 1u);
    CHECK(extracted.back().isApprox(published_orientation));
    REQUIRE(built_radians.size() == 1u);
    reaches(built_radians.back(), published_euler_degrees);
}

TEST_CASE("the orientation a tool jog window jogs by reaches the frame seam as the same orientation in radians", "[manipulator][controls]")
{
    extracted.clear();
    built_radians.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed = compose(loop, jogging(), rigid_motion::baseline().screw);
    placed.publishing->publish(chosen_pose());
    tool_jog_window panel("Tool frame jog", placed.seen, placed.owned, framing(), std::make_shared<edited_pose>(), {mode::preview});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    press_tool_frame_reset(frames, draw);
    enter_last_jog_angle(frames, draw);
    static_cast<void>(loop.drain());

    // One jog runs two conversions through this slot, in the order the handler's own statements fix:
    // the pose the jog starts from and then the jog itself. Nothing in the recorded rows says which
    // is which, so an edit that swapped those statements would swap these two assertions.
    REQUIRE(built_radians.size() >= 2u);
    reaches(built_radians[built_radians.size() - 2u], published_euler_degrees);
    reaches(built_radians.back(), jog_euler_degrees);
}
