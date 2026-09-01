#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_WORLD_OBJECT_WINDOW_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_WORLD_OBJECT_WINDOW_H

#include "praxis/manipulator/option_cycle.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include "praxis/rigid_motion/frame.h"

#include <Eigen/Core>

#include <string>
#include <memory>
#include <vector>
#include <cstdint>
#include <string_view>

namespace praxis::manipulator {

class world_object_window : public scene::imgui_window, public config::configurable
{
public:
    enum class world_view : std::uint8_t
    {
        load_stl,
        transform
    };

    struct settings
    {
        bool active;
        std::string model_path;
        world_view selected_view;
        Eigen::Vector3f gfx_scale;
        Eigen::Vector3f gfx_offset;
        Eigen::Vector3f gfx_euler_zyx_degrees;

        explicit settings(bool chosen_active = false, std::string chosen_model_path = std::string(), world_view chosen_view = world_view::load_stl,
                          const Eigen::Vector3f &chosen_gfx_scale = Eigen::Vector3f::Ones(), const Eigen::Vector3f &chosen_gfx_offset = Eigen::Vector3f::Zero(),
                          const Eigen::Vector3f &chosen_gfx_euler_zyx_degrees = Eigen::Vector3f::Zero());
    };

    world_object_window(std::string name, loadable_robot_stencil &target, const rigid_motion::frame_ops &injected);
    world_object_window(std::string name, loadable_robot_stencil &target, const rigid_motion::frame_ops &injected, const settings &state, std::string at = std::string());

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
    char m_model_path[1024];
    std::string m_settings_at;
    Eigen::Vector3f m_gfx_scale;
    Eigen::Vector3f m_gfx_offset;
    Eigen::Vector3f m_gfx_euler_zyx_degrees;
    option_cycle<world_view, 2> m_world_view;
    // The view one of this window's own controls put there. A pane the window opened for itself
    // where it holds no mesh is an accommodation and not a view anybody chose.
    world_view m_chosen_view;
    loadable_robot_stencil &m_stencil;
    rigid_motion::frame_ops m_frame;
    std::shared_ptr<threepp::Object3D> m_world_object;

    void render_activation();
    void render_stl_loader();
    void render_graphics_transform();

    void assign_gfx_transform();

    void activate_loaded_object();
    void deactivate_loaded_object();
    void clear_loaded_object();

    bool load_stl();
};

}

#endif
