#include "praxis/rigid_motion/frame_window.h"

#include "praxis/scene/widgets.h"

#include <cstddef>
#include <optional>
#include <string_view>

namespace praxis::rigid_motion {

namespace {

constexpr float position_step      = 0.05f;
constexpr float position_step_fast = 0.25f;
constexpr float angle_limit        = 180.f;

std::optional<std::size_t> named_frame(std::size_t entry)
{
    if(entry == 0)
        return std::nullopt;

    return entry - 1;
}

std::size_t entry_of(std::optional<std::size_t> frame)
{
    return frame ? *frame + 1 : 0;
}

}

void frame_window::render()
{
    if(m_objects.size() != m_stencil.count())
        resynchronize();

    ImGui::Begin(display_name().c_str());
    if(m_selecting)
        render_told_object();
    else
        for(std::size_t index = m_controls.first_panelled_object; index < m_objects.size(); ++index)
            render_panel(index);
    render_refusal();
    ImGui::End();
}

// An index past the object set draws no panel and leaves the one this window stands on where it
// was, which is the rule the stencil's own accessors state for an index outside that set.
void frame_window::render_told_object()
{
    const std::size_t told = m_selecting();
    if(told >= m_objects.size())
        return;

    m_selected = told;
    render_panel(m_selected);
}

void frame_window::render_panel(std::size_t index)
{
    ImGui::NewLine();
    ImGui::PushID(static_cast<int>(index));
    render_object(index);
    ImGui::PopID();
}

void frame_window::render_refusal()
{
    if(!m_refused.empty())
        ImGui::TextUnformatted(m_refused.c_str());
}

void frame_window::render_object(std::size_t index)
{
    const char *const angle_labels[3]{"A", "B", "C"};
    const char *const position_labels[3]{"X", "Y", "Z"};
    const std::string_view named = m_stencil.name_of(index);
    const auto reassign          = [this, index](int) { assign_pose(index); };

    ImGui::Text("Frame: %.*s", static_cast<int>(named.size()), named.data());
    if(m_controls.visibility)
        render_visibility(index);
    if(m_controls.parent)
        render_parent(index);
    if(scene::render_enum_selection("Axis order", m_objects[index].order, axis_order_labels()))
        assign_pose(index);
    scene::render_float3_slider_with_reset(m_objects[index].euler_degrees, angle_labels, -angle_limit, angle_limit, reassign);
    scene::render_float3_inputs(m_objects[index].position, position_labels, position_step, position_step_fast, reassign);
}

void frame_window::render_parent(std::size_t index)
{
    std::size_t selected = entry_of(m_objects[index].parent);
    if(scene::render_dropdown_selection("Parent", selected, m_entries))
        apply_parent(index, named_frame(selected));
}

void frame_window::render_visibility(std::size_t index)
{
    bool shown = m_stencil.axes_shown(index);
    if(ImGui::Checkbox("Axes", &shown))
        m_stencil.set_axes_shown(index, shown);
}

}
