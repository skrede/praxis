#include "window_stage.h"

#include "praxis/manipulator/task_space_window.h"
#include "praxis/manipulator/joint_control_window.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <imgui.h>

#include <chrono>
#include <memory>
#include <cstdint>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

using shape = task_space_window::motion_shape;
using mode  = control_mode;

const rigid_motion::frame_ops reference = rigid_motion::baseline().frame;

// Inside the position slider's range and equal to neither the published configuration nor the zeros
// an untouched joint panel holds, so an arm carrying it was driven from the task space panel.
constexpr const char *typed_offset = "0.6";
constexpr double slider_offset     = 0.6;

// Reached by a motion rather than by a preview, and outside the joint limits of neither joint.
const Eigen::Vector3d moved_to{0.4, -0.3, 0.0};

time_point dictated{};

time_point dictated_reading()
{
    return dictated;
}

clock_source dictating()
{
    dictated = time_point{};
    return clock_source{&dictated_reading};
}

// One service covers every period the raised reading spans, so the advance may be coarser than the
// period a playback is registered at.
constexpr seconds serviced{0.1};
constexpr std::uint32_t most_services = 20000;

composed_arm composing(praxis::scheduler::scheduler &loop)
{
    const strand work    = *loop.make_strand();
    const auto driven    = std::make_shared<scene_robot>(two_joint_arm(robot_ops{}));
    const auto published = std::make_shared<arm_publisher>();
    const auto control   = std::make_shared<robot_controller>(*driven, composing_motion(), composing_path(), task_trajectory_ops{}, composing_time_scaling(),
                                                              praxis::trajectory::trajectory_ops{}, praxis::rigid_motion::screw_ops{});

    return composed_arm{published->reader(), published, std::make_shared<owned_arm>(work, work, driven, control, published)};
}

bool played_out(praxis::scheduler::scheduler &loop, const arm_reader &seen)
{
    for(std::uint32_t taken = 0; taken < most_services && seen.read()->executing; ++taken)
    {
        dictated += std::chrono::duration_cast<time_point::duration>(serviced);
        if(!loop.drain().has_value())
            return false;
    }

    return !seen.read()->executing;
}

drawing over_both(scene::imgui_window &first, scene::imgui_window &second, const char *focused)
{
    return [&first, &second, focused]
    {
        first.render();
        second.render();
        ImGui::SetWindowFocus(focused);
    };
}

// The arm's own configuration, read on the strand that owns it rather than off the publication.
joint_vector held_by(const composed_arm &placed, praxis::scheduler::scheduler &loop)
{
    const auto read = std::make_shared<joint_vector>();
    command(placed.owned, [read](robot_controller &control, scene_robot &) { *read = control.joint_positions(); });
    static_cast<void>(loop.drain());

    return *read;
}

// The first position control stands two rows under the pane's first, which the two cycles occupy.
void enter_first_offset(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_Home);
    tap(frames, draw, ImGuiKey_DownArrow);
    tap(frames, draw, ImGuiKey_DownArrow);
    type_at_cursor(frames, draw, typed_offset);
}

// The pane's last row carries the seeding control and the move control side by side, and a row is
// entered at its leftmost.
void press_move(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_Space);
}

// Frames nobody touched. Every panel on screen still draws, which is the whole subject: what a
// panel does on a frame none of its own controls moved.
void draw_untouched(imgui_frame &frames, const drawing &draw)
{
    frames.draw_frame(draw);
    frames.draw_frame(draw);
}

std::shared_ptr<edited_pose> holding(const Eigen::Vector3d &position)
{
    auto held      = std::make_shared<edited_pose>();
    held->position = position.cast<float>();

    return held;
}

arm_snapshot upright()
{
    return at_rest(configuration(0.0, 0.0), Eigen::Vector3d::Zero(), rotation::Identity());
}

// Joint by joint rather than as a whole, so a failure names the configuration the arm was found in
// alongside the one the case drove it to.
void holds(const joint_vector &found, const joint_vector &commanded)
{
    REQUIRE(found.size() == commanded.size());
    for(Eigen::Index joint = 0; joint < commanded.size(); ++joint)
        CHECK(found[joint] == Catch::Approx(commanded[joint]).margin(round_trip));
}

}

TEST_CASE("a joint control window nobody touched leaves a task space preview standing", "[manipulator][controls]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const composed_arm placed = composing(loop);
    placed.publishing->publish(std::make_shared<const arm_snapshot>(upright()));

    joint_control_window idle("Joint control", placed.seen, placed.owned, joint_control_window::settings{mode::preview});
    task_space_window driven("Task space", placed.seen, placed.owned, reference, std::make_shared<edited_pose>(), {shape::ptp, mode::preview});

    imgui_frame frames;
    start_navigating(frames, over_both(idle, driven, "Task space"));
    enter_first_offset(frames, over_both(idle, driven, "Task space"));
    static_cast<void>(loop.drain());
    draw_untouched(frames, over_both(idle, driven, "Task space"));

    holds(held_by(placed, loop), configuration(slider_offset, 0.0));
}

TEST_CASE("a joint control window nobody touched does not take the arm back when a motion finishes", "[manipulator][controls]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const composed_arm placed = composing(loop);
    placed.publishing->publish(std::make_shared<const arm_snapshot>(upright()));

    joint_control_window idle("Joint control", placed.seen, placed.owned, joint_control_window::settings{mode::preview});
    task_space_window driven("Task space", placed.seen, placed.owned, reference, holding(moved_to), {shape::ptp, mode::simulation});

    imgui_frame frames;
    start_navigating(frames, over_both(idle, driven, "Task space"));
    press_move(frames, over_both(idle, driven, "Task space"));
    static_cast<void>(loop.drain());

    REQUIRE(placed.seen.read()->executing);
    REQUIRE(played_out(loop, placed.seen));
    draw_untouched(frames, over_both(idle, driven, "Task space"));

    holds(held_by(placed, loop), configuration(moved_to.x(), moved_to.y()));
}
