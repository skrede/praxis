#ifndef HPP_GUARD_PRAXIS_SCENE_WINDOWS_VIEW_GIZMO_WINDOW_H
#define HPP_GUARD_PRAXIS_SCENE_WINDOWS_VIEW_GIZMO_WINDOW_H

#include "praxis/scene/imgui_window.h"

#include <threepp/cameras/Camera.hpp>

#include <threepp/controls/OrbitControls.hpp>

#include <array>
#include <string>
#include <cstdint>
#include <optional>

namespace praxis::scene {

struct view_gizmo_axis_style
{
    char label;
    ImVec4 color;
    bool reversed;
};

using view_gizmo_axis_styles = std::array<view_gizmo_axis_style, 3>;

view_gizmo_axis_styles default_view_gizmo_axis_styles();

class view_gizmo_window : public imgui_window
{
    struct drag_state
    {
        bool positive;
        std::uint8_t axis;
        ImVec2 screen_direction;
        threepp::Vector3 target;
        threepp::Vector3 camera_position;
    };

public:
    view_gizmo_window(std::string name, threepp::Camera &camera, threepp::OrbitControls &controls, view_gizmo_axis_styles styles = default_view_gizmo_axis_styles());

    void render() override;

private:
    // The GUI library's double-hash separator leaves the visible label empty and makes the rest the identity.
    std::string m_panel_title;
    threepp::Camera &m_camera;
    std::optional<drag_state> m_drag;
    view_gizmo_axis_styles m_axis_styles;
    threepp::OrbitControls &m_controls;

    void snap_view(std::uint8_t axis, bool positive);

    void update_drag(const ImVec2 &delta);

    void handle_interaction(bool hot, std::uint8_t axis, bool positive);
};

}

#endif
