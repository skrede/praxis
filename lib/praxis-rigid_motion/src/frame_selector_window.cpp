#include "praxis/rigid_motion/frame_selector_window.h"

#include "praxis/scene/widgets.h"

#include <imgui.h>

#include <string>
#include <vector>
#include <cstddef>
#include <utility>

namespace praxis::rigid_motion {

namespace {

// The objects of the stencil in the stencil's own order, and nothing beside them: the frame they
// hang under takes no index here, so an entry's place in this list is the index it names.
std::vector<std::string> object_entries(const frame_stencil &target)
{
    std::vector<std::string> entries;
    entries.reserve(target.count());
    for(std::size_t index = 0; index < target.count(); ++index)
        entries.emplace_back(target.name_of(index));

    return entries;
}

}

frame_selector_window::frame_selector_window(std::string name, frame_stencil &target)
        : imgui_window(std::move(name))
        , m_selected(0)
        , m_stencil(target)
        , m_entries(object_entries(target))
{
}

void frame_selector_window::select_object(std::size_t index)
{
    const std::size_t carried = m_stencil.count();
    if(carried == 0)
        m_selected = 0;
    else
        m_selected = index < carried ? index : carried - 1;
}

void frame_selector_window::render()
{
    if(m_entries.size() != m_stencil.count())
        resynchronize();

    ImGui::Begin(display_name().c_str());
    render_selected_object();
    render_axes_shown();
    ImGui::End();
}

void frame_selector_window::render_selected_object()
{
    scene::render_dropdown_selection("Object", m_selected, m_entries);
}

void frame_selector_window::render_axes_shown()
{
    if(m_stencil.count() == 0)
        return;

    bool shown = m_stencil.axes_shown(m_selected);
    if(ImGui::Checkbox("Axes", &shown))
        m_stencil.set_axes_shown(m_selected, shown);
}

// Asked only when the object set has changed size, which is the signal a placement panel follows
// too, and the choice is brought back inside the set it names into.
void frame_selector_window::resynchronize()
{
    m_entries = object_entries(m_stencil);
    if(m_entries.empty())
        m_selected = 0;
    else if(m_selected >= m_entries.size())
        m_selected = m_entries.size() - 1;
}

}
