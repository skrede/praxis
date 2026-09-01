#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_TOOL_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_TOOL_WINDOW_H

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/option_cycle.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/axis_order.h"

#include <Eigen/Core>

#include <string>
#include <memory>
#include <vector>
#include <cstdint>
#include <string_view>

namespace praxis::manipulator {

class tool_window : public scene::imgui_window, public config::configurable
{
public:
    enum class tool_view : std::uint8_t
    {
        kinematics_transform,
        graphics_transform,
        load_stl
    };

    struct settings
    {
        bool active;
        std::string model_path;
        tool_view selected_view;
        Eigen::Vector3f gfx_euler_degrees;
        axis_order gfx_euler_order;
        Eigen::Vector3f gfx_scale;
        Eigen::Vector3f gfx_offset;
        Eigen::Vector3f kinematics_euler_degrees;
        axis_order kinematics_euler_order;
        Eigen::Vector3f kinematics_offset;

        explicit settings(bool chosen_active = false, std::string chosen_model_path = std::string(), tool_view chosen_view = tool_view::load_stl,
                          const Eigen::Vector3f &chosen_gfx_euler_degrees = Eigen::Vector3f::Zero(), axis_order chosen_gfx_euler_order = axis_order::zyx,
                          const Eigen::Vector3f &chosen_gfx_scale = Eigen::Vector3f::Ones(), const Eigen::Vector3f &chosen_gfx_offset = Eigen::Vector3f::Zero(),
                          const Eigen::Vector3f &chosen_kinematics_euler_degrees = Eigen::Vector3f::Zero(), axis_order chosen_kinematics_euler_order = axis_order::zyx,
                          const Eigen::Vector3f &chosen_kinematics_offset = Eigen::Vector3f::Zero());
    };

    tool_window(std::string name, loadable_robot_stencil &stencil, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected);
    tool_window(std::string name, loadable_robot_stencil &stencil, arm_reader seen, std::weak_ptr<owned_arm> arm, const rigid_motion::frame_ops &injected, const settings &state,
                std::string at = std::string());

    settings state() const;

    void render() override;

    void initialize() override;

    std::string_view settings_path() const override
    {
        return m_settings_at;
    }

    std::vector<config::edit> settings_edits(const config::document &) const override;

    // A window no key path was named for has nowhere to write, so it offers nothing.
    const config::configurable *as_configurable() const override
    {
        return m_settings_at.empty() ? nullptr : this;
    }

private:
    bool m_active;
    arm_reader m_seen;
    char m_model_path[1024];
    std::string m_settings_at;
    Eigen::Vector3f m_gfx_euler_degrees;
    Eigen::Vector3f m_gfx_scale;
    Eigen::Vector3f m_gfx_offset;
    Eigen::Vector3f m_tool_euler_degrees;
    axis_order m_gfx_euler_order;
    Eigen::Vector3f m_tool_offset;
    axis_order m_tool_euler_order;
    std::weak_ptr<owned_arm> m_arm;
    option_cycle<tool_view, 3> m_tool_view;
    // The view one of this window's own controls put there. A pane the window opened for itself
    // where it holds no mesh is an accommodation and not a view anybody chose.
    tool_view m_chosen_view;
    loadable_robot_stencil &m_stencil;
    std::shared_ptr<threepp::Object3D> m_tool;
    rigid_motion::frame_ops m_frame;

    void render_activation();
    void render_stl_loader();
    void render_graphics_transform();
    void render_kinematics_transform();

    void assign_gfx_transform();
    void assign_kinematics_transform();

    void seat_attached_tool();

    void activate_custom_tool();
    void activate_default_tool();

    bool load_stl();
};

}

#endif
