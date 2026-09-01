#include "windows/preset_window.h"

#include "praxis/scene/widgets.h"

#include <string>
#include <vector>
#include <utility>

namespace praxis::scene {

preset_window::preset_window(std::string name, visualizer &v)
        : imgui_window(std::move(name))
        , m_remember(false)
        , m_selected(0)
        , m_visualizer(v)
        , m_requested()
{
}

void preset_window::render()
{
    ImGui::Begin(display_name().c_str());
    if(m_visualizer.awaiting_answer())
        render_question();
    else
        render_body();
    ImGui::End();
}

void preset_window::render_body()
{
    take_requested();

    const bool offered = render_preset_selection();
    if(!m_visualizer.is_preset_loaded())
        return;

    if(offered)
        ImGui::SameLine();
    if(ImGui::Button("Unload"))
        m_visualizer.clear_preset();
    if(!m_visualizer.save_offered())
        return;

    ImGui::SameLine();
    if(ImGui::Button("Save"))
        m_visualizer.save_values();
}

// Drawn in place of the ordinary body, so nothing can be composed, released or written while a
// question stands. Neither answer is preselected, no key reaches either, and closing the window
// answers nothing.
void preset_window::render_question()
{
    ImGui::TextUnformatted("What is composed differs from what the file carries.");
    ImGui::Checkbox("Remember this answer", &m_remember);
    if(ImGui::Button("Keep"))
        answer(true);
    ImGui::SameLine();
    if(ImGui::Button("Discard"))
        answer(false);
}

void preset_window::take_requested()
{
    if(!m_requested.has_value() || m_visualizer.is_preset_loaded())
        return;

    const std::string next = *m_requested;
    m_requested.reset();
    m_visualizer.load_preset(next);
}

// What is held is released first and the request waits for that, which is what makes a replacing
// change ask the same question an outright release asks.
void preset_window::request(std::string name)
{
    m_requested = std::move(name);
    if(m_visualizer.is_preset_loaded())
        m_visualizer.clear_preset();
}

void preset_window::answer(bool keep)
{
    m_visualizer.answer(leaving_answer{keep, m_remember});
    m_remember = false;
}

// The selection is an index into the registry's name list, which a load does not change; the clamp
// covers a list that has shrunk under it.
bool preset_window::render_preset_selection()
{
    const std::vector<std::string> presets = m_visualizer.preset_names();
    if(presets.empty())
        return false;
    if(m_selected >= presets.size())
        m_selected = 0;

    if(render_dropdown_selection("##item", m_selected, presets))
        m_requested.reset();
    ImGui::SameLine();
    if(ImGui::Button("Load"))
        request(presets[m_selected]);

    return true;
}

}
