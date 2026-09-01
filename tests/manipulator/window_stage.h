#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_WINDOW_STAGE_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_WINDOW_STAGE_H

#include "fixtures.h"

#include "imgui_frame.h"
#include "panel_keys.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/robot_controller.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/screw.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <Eigen/Core>

#include <memory>
#include <cstddef>
#include <functional>

namespace praxis::fixture {

inline const char *const absent_line = "The arm has published nothing yet.";

// A clock that never advances, so no posted deadline falls due and a drain returns once the work a
// frame put on the strand has run.
inline scheduler::time_point reading()
{
    return scheduler::time_point{};
}

struct composed_arm
{
    arm_reader seen;
    std::shared_ptr<arm_publisher> publishing;
    std::shared_ptr<owned_arm> owned;
};

inline composed_arm compose(scheduler::scheduler &loop, const motion_ops &moving, const rigid_motion::screw_ops &turning)
{
    const trajectory::path_ops along{.joint_straight_line = &straight_line, .screw = &interpolated, .decoupled = &interpolated};
    const scheduler::strand work = *loop.make_strand();
    const auto driven            = std::make_shared<scene_robot>(two_joint_arm(robot_ops{}));
    const auto published         = std::make_shared<arm_publisher>();
    const auto control           = std::make_shared<robot_controller>(*driven, moving, along, task_trajectory_ops{}, composing_time_scaling(), trajectory::trajectory_ops{}, turning);

    return composed_arm{published->reader(), published, std::make_shared<owned_arm>(work, work, driven, control, published)};
}

// Only the joints and the tool pose reach a control window, so the rest of a publication is what an
// arm at rest reports: every other pose at the origin and every other orientation upright.
inline arm_snapshot at_rest(const joint_vector &joints, const expected<Eigen::Vector3d, refusal> &position, const expected<rotation, refusal> &orientation)
{
    const transform identity = transform::Identity();

    return arm_snapshot{joints,
                        joint_limits{},
                        identity,
                        identity,
                        identity,
                        position,
                        position,
                        orientation,
                        orientation,
                        recording_parameters{},
                        1.0,
                        false,
                        scheduler::task_counters{},
                        {},
                        unexpected(refusal::not_implemented),
                        unexpected(refusal::not_implemented),
                        jacobian_manipulability{unexpected(refusal::not_implemented), unexpected(refusal::not_implemented)},
                        jacobian_manipulability{unexpected(refusal::not_implemented), unexpected(refusal::not_implemented)},
                        {},
                        {},
                        nullptr,
                        nullptr,
                        {},
                        {}};
}

inline std::shared_ptr<arm_publisher> publishing(const arm_snapshot &seen)
{
    auto published = std::make_shared<arm_publisher>();
    published->publish(std::make_shared<const arm_snapshot>(seen));

    return published;
}

// A window drawing one panel is focused by its own title, which is the panel's identity. The focus
// request is issued inside the frame because a panel the frame has not opened yet cannot be focused.
inline drawing over(scene::imgui_window &panel)
{
    return [&panel]
    {
        panel.render();
        ImGui::SetWindowFocus(panel.display_name().c_str());
    };
}

// Every vertex one drawing produced, so two panels are compared by what they put on screen rather
// than by how much of it there was.
inline std::size_t geometry_of(const drawing &draw)
{
    tests::imgui_frame frames;
    frames.draw(draw);

    REQUIRE(frames.has_draw_data());
    REQUIRE(frames.command_lists() == 1);
    REQUIRE(frames.vertices() > 0);

    return frames.signature();
}

// A panel drawn over an absent publication is the panel that title would carry with that one
// sentence in it and nothing else, so what the two frames drew is the whole comparison.
inline std::size_t stating_absence(const char *title)
{
    return geometry_of(
            [title]
            {
                ImGui::Begin(title);
                ImGui::TextUnformatted(absent_line);
                ImGui::End();
            });
}

// The first key a focused panel is given only raises the keyboard cursor, so the absolute step is
// taken twice: the second lands where the first would have whether or not the first was spent.
inline void reach(tests::imgui_frame &frames, const drawing &draw, ImGuiKey step)
{
    tap(frames, draw, step);
    tap(frames, draw, step);
}

inline void start_navigating(tests::imgui_frame &frames, const drawing &draw)
{
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    frames.draw(draw);
}

// The enter key is what opens a slider or a field for typed input; the activation key steps it by
// the library's own increment instead. The opened field holds its text selected, so the first
// character replaces the formatted value rather than appending to it, and a temp input applies its
// parsed value on every frame its buffer changes.
inline void type_at_cursor(tests::imgui_frame &frames, const drawing &draw, const char *degrees_text)
{
    tap(frames, draw, ImGuiKey_Enter);
    for(const char *at = degrees_text; *at != '\0'; ++at)
    {
        ImGui::GetIO().AddInputCharacter(static_cast<unsigned int>(*at));
        frames.draw_frame(draw);
    }
    tap(frames, draw, ImGuiKey_Enter);
}

}

#endif
