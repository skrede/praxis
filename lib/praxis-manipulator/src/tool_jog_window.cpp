#include "praxis/manipulator/option_widgets.h"
#include "praxis/manipulator/tool_jog_window.h"
#include "praxis/manipulator/control_configuration.h"

#include "praxis/extension/coverage.h"
#include "praxis/extension/held_handle.h"

#include "praxis/scene/widgets.h"

#include "praxis/rigid_motion/slots.h"
#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/axis_order.h"

#include <imgui.h>

#include <spdlog/spdlog.h>

#include <Eigen/Core>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <string_view>

namespace praxis::manipulator {

namespace {

constexpr std::array<control_mode, 2> mode_options{control_mode::preview, control_mode::simulation};

const char *const position_labels[3]{"X", "Y", "Z"};

constexpr const char *unpublished_arm = "The arm has published nothing yet.";

const char *const orientation_labels[3]{"A", "B", "C"};

rotation turn_step(const rigid_motion::frame_ops &frames, int axis, double radians, axis_order order)
{
    return frames.rotation_matrix_from_euler(Eigen::Vector3d::Unit(axis) * radians, order);
}

transform stepped_along(const rigid_motion::frame_ops &frames, const transform &start, int axis, float metres)
{
    return start * frames.transformation_matrix_from_rotation_position(rotation::Identity(), Eigen::Vector3d::Unit(axis) * metres);
}

transform stepped_about(const rigid_motion::frame_ops &frames, const transform &start, int axis, float degrees, axis_order order)
{
    return start * frames.transformation_matrix_from_rotation_position(turn_step(frames, axis, degrees * radians_per_degree, order), Eigen::Vector3d::Zero());
}

bool declined(std::string_view slot, std::string_view reason)
{
    spdlog::error("praxis: '{}' {}, so the turn is not kept and the tool returns to the pose the window holds", slot, reason);

    return false;
}

}

tool_jog_window::settings::settings(control_mode chosen)
        : mode(chosen)
{
}

tool_jog_window::tool_jog_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, std::shared_ptr<edited_pose> edited)
        : tool_jog_window(std::move(name), std::move(seen), std::move(arm), injected, std::move(edited), settings{})
{
}

tool_jog_window::tool_jog_window(std::string name, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, std::shared_ptr<edited_pose> edited,
                                 const settings &state, std::string at)
        : imgui_window(std::move(name))
        , m_seen(seen)
        , m_settings_at(std::move(at))
        , m_arm(std::move(arm))
        , m_jog_position(Eigen::Vector3f::Zero())
        , m_frame(injected)
        , m_jog_euler_degrees(Eigen::Vector3f::Zero())
        , m_edited(std::move(edited))
        , m_control_mode(state.mode, mode_options, control_mode_labels())
{
    static_cast<void>(held(seen.read(), "the tool jog window", "published arm state"));
    static_cast<void>(held(m_edited, "the tool jog window", "shared pose"));
}

tool_jog_window::settings tool_jog_window::state() const
{
    return settings{m_control_mode.value()};
}

std::vector<config::edit> tool_jog_window::settings_edits(const config::document &carried) const
{
    return config::unsaved_edits(carried, write_tool_jog(state(), m_settings_at));
}

void tool_jog_window::render()
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();

    ImGui::Begin(display_name().c_str());
    if(published == nullptr)
        ImGui::TextUnformatted(unpublished_arm);
    else
    {
        render_option_cycle("Control mode", m_control_mode);
        render_tool_frame_jog(*published);
    }
    ImGui::End();
}

bool tool_jog_window::render_jog_start_pose(const arm_snapshot &seen)
{
    seed_unless_held(*m_edited, seen, m_frame);

    bool moved = ImGui::InputFloat3("XYZ", m_edited->position.data());
    moved      = ImGui::InputFloat3("ABC", m_edited->euler_degrees.data()) || moved;
    moved      = scene::render_enum_selection("Euler order", m_edited->order, axis_order_labels()) || moved;

    if(moved)
        m_edited->standing = pose_standing::held;

    if(ImGui::Button("Reset"))
        static_cast<void>(seed_from(*m_edited, seen, m_frame));

    return moved;
}

void tool_jog_window::preview(const transform &tool_pose)
{
    if(m_control_mode == control_mode::preview)
        command(m_arm, [tool_pose](robot_controller &control, scene_robot &) { control.preview_task_space_pose(tool_pose); });
}

void tool_jog_window::render_tool_frame_jog(const arm_snapshot &seen)
{
    if(render_jog_start_pose(seen))
        preview(pose_matrix(*m_edited, m_frame));

    const auto offset = [this](int axis) { return stepped_along(m_frame, pose_matrix(*m_edited, m_frame), axis, m_jog_position[axis]); };
    const auto turn   = [this](int axis) { return stepped_about(m_frame, pose_matrix(*m_edited, m_frame), axis, m_jog_euler_degrees[axis], m_edited->order); };

    scene::render_float3_slider(m_jog_position, position_labels, -1.f, 1.f, [&](int axis) { preview(offset(axis)); }, [&](int axis) { release_step(offset(axis), false); });
    scene::render_float3_slider(m_jog_euler_degrees, orientation_labels, -180.f, 180.f, [&](int axis) { preview(turn(axis)); }, [&](int axis) { release_step(turn(axis), true); });
}

void tool_jog_window::release_step(const transform &reached, bool turned)
{
    m_jog_position      = Eigen::Vector3f::Zero();
    m_jog_euler_degrees = Eigen::Vector3f::Zero();
    if(m_control_mode != control_mode::preview)
        return;

    if(turned && !fold_orientation(reached))
    {
        preview(pose_matrix(*m_edited, m_frame));
        return;
    }

    // A released step is where the next one starts.
    m_edited->position = reached.topRightCorner<3, 1>().cast<float>();
    m_edited->standing = pose_standing::held;
}

bool tool_jog_window::fold_orientation(const transform &reached)
{
    const auto slot            = static_cast<std::size_t>(rigid_motion::frame_slot::euler_from_rotation_matrix);
    const capability_view view = rigid_motion::view_of(m_frame);
    if(holds_default(view, slot))
        return declined(slot_name(view, slot), "holds its default");

    const Eigen::Vector3d angles = m_frame.euler_from_rotation_matrix(reached.topLeftCorner<3, 3>(), m_edited->order);
    if(!angles.allFinite())
        return declined(slot_name(view, slot), "answered angles that are not finite");

    m_edited->euler_degrees = (angles * degrees_per_radian).cast<float>();

    return true;
}

}
