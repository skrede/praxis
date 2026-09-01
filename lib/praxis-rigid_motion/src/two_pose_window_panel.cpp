#include "praxis/rigid_motion/two_pose_window.h"

#include "praxis/scene/widgets.h"

#include <imgui.h>

namespace praxis::rigid_motion {

namespace {

constexpr float position_step      = 0.05f;
constexpr float position_step_fast = 0.25f;
constexpr float angle_limit        = 180.f;

}

void two_pose_window::render()
{
    ImGui::Begin(display_name().c_str());
    ImGui::Text("Start pose");
    render_pose(m_shown.start, 0);
    ImGui::Text("End pose");
    render_pose(m_shown.end, 1);
    ImGui::SliderFloat("Travelled", &m_shown.parameter, 0.f, 1.f);
    render_reading();
    ImGui::End();

    settle();
}

void two_pose_window::render_pose(pose_controls &shown, int id)
{
    const char *const angle_labels[3]{"A", "B", "C"};
    const char *const position_labels[3]{"X", "Y", "Z"};

    ImGui::PushID(id);
    scene::render_enum_selection("Axis order", shown.order, axis_order_labels());
    scene::render_float3_slider(shown.euler_degrees, angle_labels, -angle_limit, angle_limit);
    scene::render_float3_inputs(shown.position, position_labels, position_step, position_step_fast);
    ImGui::PopID();
}

void two_pose_window::render_reading()
{
    const scene::readout shown = reading();
    if(!shown.message.empty())
        return ImGui::TextUnformatted(shown.message.c_str());

    ImGui::Text("Travelled %.3f m", static_cast<double>(shown.rows.front().front().value));
}

}
