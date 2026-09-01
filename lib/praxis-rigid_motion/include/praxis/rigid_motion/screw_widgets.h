#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_SCREW_WIDGETS_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_SCREW_WIDGETS_H

#include <imgui.h>

#include <Eigen/Core>

#include <functional>

namespace praxis::rigid_motion {

// The widget call is the left operand of every disjunction below: the right one goes unevaluated
// once a control has moved, and a control not drawn is a control not on screen.

// The point of the axis, its direction and the pitch, which is the axis translation per radian. The
// control given is drawn on the point's own row and answers whether it moved the point.
inline bool render_screw_inputs(Eigen::Vector3f &point, Eigen::Vector3f &direction, float &pitch, const std::function<bool()> &beside_point = nullptr)
{
    bool moved = ImGui::InputFloat3("q", point.data());
    if(beside_point != nullptr)
    {
        ImGui::SameLine();
        moved = beside_point() || moved;
    }
    moved = ImGui::InputFloat3("w", direction.data()) || moved;
    ImGui::SameLine();
    if(ImGui::Button("Normalize"))
    {
        direction.normalize();
        moved = true;
    }

    return ImGui::InputFloat("h", &pitch) || moved;
}

// The angular part and the linear part, each per unit of the angle the two are turned through.
inline bool render_twist_inputs(Eigen::Vector3f &angular_part, Eigen::Vector3f &linear_part)
{
    bool moved = ImGui::InputFloat3("w", angular_part.data());
    ImGui::SameLine();
    if(ImGui::Button("Normalize"))
    {
        angular_part.normalize();
        moved = true;
    }

    return ImGui::InputFloat3("v", linear_part.data()) || moved;
}

// The direction an axis runs along, with the control that makes it a unit vector on its own row.
inline bool render_direction_input(Eigen::Vector3f &direction)
{
    bool moved = ImGui::InputFloat3("w", direction.data());
    ImGui::SameLine();
    if(ImGui::Button("Normalize"))
    {
        direction.normalize();
        moved = true;
    }

    return moved;
}

}

#endif
