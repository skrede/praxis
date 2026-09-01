#include "window_stage.h"

#include "praxis/manipulator/screw_jog_window.h"

#include "praxis/rigid_motion/angles.h"
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

const rigid_motion::screw_ops reference_screw = rigid_motion::baseline().screw;

using mode = control_mode;

// Neither a right angle nor a straight one, so a conversion applied twice and a conversion applied
// in the opposite direction each land somewhere the correct value is not.
constexpr const char *typed_degrees  = "37";
constexpr double screw_angle_degrees = 37.0;

// The control holds the angle in single precision, so the value reaching the seam is the entered
// degrees to within one float step of them, which is 6.7e-8 radians at this angle.
constexpr double conversion_tolerance = 1.0e-7;

std::vector<double> previewed;
std::vector<double> executed;

expected<joint_vector, refusal> recorded_preview(const rigid_motion::screw_ops &, const kinematics &, const transform &, const Eigen::Vector3d &, const Eigen::Vector3d &,
                                                 double theta_radians, double, const joint_vector &j0)
{
    previewed.push_back(theta_radians);

    return j0;
}

transform recorded_exponential(const screw_axis &axis, double theta_radians)
{
    executed.push_back(theta_radians);

    return reference_screw.matrix_exponential_screw(axis, theta_radians);
}

motion_ops previewing()
{
    return motion_ops{.task_space_pose = &position_as_configuration, .task_space_screw = &recorded_preview};
}

rigid_motion::screw_ops turning()
{
    rigid_motion::screw_ops ops  = reference_screw;
    ops.matrix_exponential_screw = &recorded_exponential;

    return ops;
}

// The angle control stands one row above the pane's last.
void enter_screw_angle(imgui_frame &frames, const drawing &draw)
{
    start_navigating(frames, draw);
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_UpArrow);
    type_at_cursor(frames, draw, typed_degrees);
}

// The pane's last row carries the reset control and the move control side by side, so the sideways
// steps settle on the rightmost of the two.
void press_move(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_Space);
}

}

TEST_CASE("an angle typed at the screw jog window's slider is previewed as the same angle in radians", "[manipulator][controls]")
{
    previewed.clear();
    executed.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed = compose(loop, previewing(), turning());
    screw_jog_window panel("Screw jog", placed.seen, placed.owned, rigid_motion::baseline().frame, std::make_shared<edited_pose>(), {mode::preview});

    imgui_frame frames;
    const drawing draw = over(panel);
    enter_screw_angle(frames, draw);
    static_cast<void>(loop.drain());

    REQUIRE(!previewed.empty());
    CHECK(executed.empty());
    CHECK(previewed.back() == Catch::Approx(screw_angle_degrees * radians_per_degree).margin(conversion_tolerance));
}

TEST_CASE("an angle typed at the screw jog window's slider is moved along as the same angle in radians", "[manipulator][controls]")
{
    previewed.clear();
    executed.clear();

    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm placed = compose(loop, previewing(), turning());
    screw_jog_window panel("Screw jog", placed.seen, placed.owned, rigid_motion::baseline().frame, std::make_shared<edited_pose>(), {mode::simulation});

    imgui_frame frames;
    const drawing draw = over(panel);
    enter_screw_angle(frames, draw);
    press_move(frames, draw);
    static_cast<void>(loop.drain());

    REQUIRE(!executed.empty());
    CHECK(previewed.empty());
    CHECK(executed.back() == Catch::Approx(screw_angle_degrees * radians_per_degree).margin(conversion_tolerance));
}
