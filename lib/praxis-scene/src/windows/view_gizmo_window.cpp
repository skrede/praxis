#include "windows/view_gizmo_window.h"

#include <threepp/math/Quaternion.hpp>

#include <array>
#include <cmath>
#include <string>
#include <cstddef>
#include <utility>
#include <algorithm>

namespace praxis::scene {

namespace {

struct axis_end
{
    float depth;
    float radius;
    bool positive;
    ImVec2 position;
    std::uint8_t axis;
};

threepp::Vector3 axis_direction(std::uint8_t axis, bool positive)
{
    threepp::Vector3 direction;
    direction[axis] = positive ? 1.f : -1.f;
    return direction;
}

ImU32 axis_color(const view_gizmo_axis_style &style, float fade, float lift, float alpha = 1.f)
{
    ImVec4 color = style.color;
    color.x      = color.x * fade + (1.f - color.x * fade) * lift;
    color.y      = color.y * fade + (1.f - color.y * fade) * lift;
    color.z      = color.z * fade + (1.f - color.z * fade) * lift;
    color.w      = alpha;
    return ImGui::ColorConvertFloat4ToU32(color);
}

ImVec2 screen_direction(const threepp::Camera &camera, std::uint8_t axis, bool positive)
{
    threepp::Quaternion inverse;
    inverse.copy(camera.quaternion).invert();
    const auto direction = axis_direction(axis, positive).applyQuaternion(inverse);
    const float length   = std::hypot(direction.x, direction.y);
    return length > 0.05f ? ImVec2{direction.x / length, -direction.y / length} : ImVec2{0.f, -1.f};
}

std::array<axis_end, 6> axis_ends(const threepp::Camera &camera, const view_gizmo_axis_styles &styles, const ImVec2 &center, float radius, float scale)
{
    threepp::Quaternion inverse;
    inverse.copy(camera.quaternion).invert();
    std::array<axis_end, 6> ends;
    std::size_t end_index = 0;
    for(std::uint8_t axis = 0; axis < std::uint8_t{3}; ++axis)
        for(bool positive : {true, false})
        {
            const auto direction          = axis_direction(axis, positive).applyQuaternion(inverse);
            const bool presented_positive = positive != styles[axis].reversed;
            ends[end_index++]             = {.depth    = direction.z,
                                             .radius   = (presented_positive ? 9.f : 7.f) * scale * (0.95f + 0.05f * direction.z),
                                             .positive = positive,
                                             .position = {center.x + direction.x * (radius - 11.5f * scale), center.y - direction.y * (radius - 11.5f * scale)},
                                             .axis     = axis};
        }
    std::sort(ends.begin(), ends.end(), [](const auto &lhs, const auto &rhs) { return lhs.depth < rhs.depth; });
    return ends;
}

std::optional<axis_end> hovered_axis(const std::array<axis_end, 6> &ends, const ImVec2 &mouse, float scale)
{
    std::optional<axis_end> result;
    for(const auto &end : ends)
    {
        const float x     = mouse.x - end.position.x;
        const float y     = mouse.y - end.position.y;
        const float reach = end.radius + 2.f * scale;
        if(x * x + y * y <= reach * reach)
            result = end;
    }
    return result;
}

void draw_axis_ends(ImDrawList &draw, const std::array<axis_end, 6> &ends, const std::optional<axis_end> &hot, const view_gizmo_axis_styles &styles, const ImVec2 &center, float scale)
{
    for(const auto &end : ends)
    {
        const auto &style = styles[end.axis];
        const float fade  = 0.55f + 0.45f * (end.depth * 0.5f + 0.5f);
        const bool is_hot = hot && hot->axis == end.axis && hot->positive == end.positive;
        const ImU32 color = axis_color(style, fade, is_hot ? 0.35f : 0.f);
        if(end.positive != style.reversed)
        {
            draw.AddLine(center, end.position, color, 2.2f * scale);
            draw.AddCircleFilled(end.position, end.radius, color);
            const char label[2]     = {style.label, '\0'};
            const ImVec2 label_size = ImGui::CalcTextSize(label);
            draw.AddText({end.position.x - label_size.x * 0.5f, end.position.y - label_size.y * 0.5f}, IM_COL32(20, 23, 26, 255), label);
        }
        else
        {
            draw.AddCircleFilled(end.position, end.radius, axis_color(style, fade, 0.f, is_hot ? 0.9f : 0.25f));
            draw.AddCircle(end.position, end.radius, color, 0, 1.6f * scale);
        }
    }
}

}

view_gizmo_axis_styles default_view_gizmo_axis_styles()
{
    return {{{'X', {1.f, 0.21f, 0.33f, 1.f}, false}, {'Z', {0.17f, 0.56f, 1.f, 1.f}, false}, {'Y', {0.54f, 0.86f, 0.f, 1.f}, true}}};
}

view_gizmo_window::view_gizmo_window(std::string name, threepp::Camera &camera, threepp::OrbitControls &controls, view_gizmo_axis_styles styles)
        : imgui_window(std::move(name))
        , m_panel_title("##" + display_name())
        , m_camera(camera)
        , m_drag(std::nullopt)
        , m_axis_styles(std::move(styles))
        , m_controls(controls)
{
}

void view_gizmo_window::render()
{
    const float scale    = ImGui::GetFontSize() / 13.f;
    const float extent   = 96.f * scale;
    const auto *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos({viewport->WorkPos.x + viewport->WorkSize.x - 12.f * scale, viewport->WorkPos.y + 12.f * scale}, ImGuiCond_Always, {1.f, 0.f});
    ImGui::SetNextWindowSize({extent, extent});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0.f, 0.f});
    constexpr ImGuiWindowFlags flags =
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoNav;
    ImGui::Begin(m_panel_title.c_str(), nullptr, flags);
    ImGui::InvisibleButton("##ViewGizmoInput", {extent, extent});
    const ImVec2 minimum = ImGui::GetItemRectMin();
    const ImVec2 center{minimum.x + extent * 0.5f, minimum.y + extent * 0.5f};
    const auto ends    = axis_ends(m_camera, m_axis_styles, center, 40.f * scale, scale);
    const bool hovered = ImGui::IsItemHovered();
    const auto hot     = hovered ? hovered_axis(ends, ImGui::GetIO().MousePos, scale) : std::nullopt;
    auto *draw         = ImGui::GetWindowDrawList();
    draw->AddCircleFilled(center, 40.f * scale, IM_COL32(12, 14, 17, hovered ? 150 : 90));
    draw_axis_ends(*draw, ends, hot, m_axis_styles, center, scale);
    handle_interaction(hot.has_value(), hot ? hot->axis : std::uint8_t{0}, hot && hot->positive);
    ImGui::End();
    ImGui::PopStyleVar();
}

void view_gizmo_window::handle_interaction(bool hot, std::uint8_t axis, bool positive)
{
    const bool presented_positive = positive != m_axis_styles[axis].reversed;
    if(hot && ImGui::IsItemHovered())
        ImGui::SetTooltip("Click: view from %s%c\nDrag: translate along %c", presented_positive ? "+" : "-", m_axis_styles[axis].label, m_axis_styles[axis].label);
    if(hot && ImGui::IsItemActivated())
        m_drag = drag_state{
                .positive = positive, .axis = axis, .screen_direction = screen_direction(m_camera, axis, positive), .target = m_controls.target, .camera_position = m_camera.position};
    if(!m_drag)
        return;

    const ImVec2 delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left, 0.f);
    if(ImGui::IsMouseDown(ImGuiMouseButton_Left))
        update_drag(delta);
    else if(ImGui::IsMouseReleased(ImGuiMouseButton_Left))
    {
        if(std::hypot(delta.x, delta.y) < 4.f)
            snap_view(m_drag->axis, m_drag->positive);
        m_drag.reset();
    }
}

void view_gizmo_window::update_drag(const ImVec2 &delta)
{
    if(std::hypot(delta.x, delta.y) < 4.f)
        return;

    const float pixels   = delta.x * m_drag->screen_direction.x + delta.y * m_drag->screen_direction.y;
    const float distance = std::max(m_drag->camera_position.distanceTo(m_drag->target), 0.1f);
    const auto direction = axis_direction(m_drag->axis, m_drag->positive);
    const float amount   = pixels * distance / 120.f;
    m_camera.position.copy(m_drag->camera_position).addScaledVector(direction, amount);
    m_controls.target.copy(m_drag->target).addScaledVector(direction, amount);
}

void view_gizmo_window::snap_view(std::uint8_t axis, bool positive)
{
    auto direction = axis_direction(axis, positive);
    threepp::Vector3 current;
    current.subVectors(m_camera.position, m_controls.target).normalize();
    if(current.dot(direction) > 0.9999f)
        direction.negate();

    const float distance = std::max(m_camera.position.distanceTo(m_controls.target), 0.1f);
    m_camera.position.copy(m_controls.target).addScaledVector(direction, distance);
    m_camera.lookAt(m_controls.target);
}

}
