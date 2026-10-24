#include "windows/preset_window.h"

#include "praxis/scene/widgets.h"

#include <ranges>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <string_view>

namespace praxis::scene {

namespace {

constexpr const char *load_label   = "Load";
constexpr const char *unload_label = "Unload";
constexpr const char *save_label   = "Save";

float framed_width(const char *label)
{
    const float padding = ImGui::GetStyle().FramePadding.x;
    return padding + ImGui::CalcTextSize(label).x + padding;
}

// The widest name, framed, beside the arrow square the combo draws at frame height. `names` is not
// empty.
float dropdown_width(const std::vector<std::string> &names)
{
    return std::ranges::max(names | std::views::transform([](const std::string &name) { return framed_width(name.c_str()); })) + ImGui::GetFrameHeight();
}

}

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
    fit_on_first_use();
    ImGui::Begin(display_name().c_str());
    if(m_visualizer.awaiting_answer())
        render_question();
    else
        render_body();
    ImGui::End();
}

// Wide enough for the whole row where no saved layout places the window; the height fits the content.
void preset_window::fit_on_first_use() const
{
    const std::vector<std::string> presets = m_visualizer.preset_names();
    if(presets.empty())
        return;

    const ImGuiStyle &style = ImGui::GetStyle();
    float row               = dropdown_width(presets) + style.ItemSpacing.x + framed_width(load_label) + style.ItemSpacing.x + framed_width(unload_label);
    if(m_visualizer.save_offered())
        row += style.ItemSpacing.x + framed_width(save_label);
    ImGui::SetNextWindowSize(ImVec2(style.WindowPadding.x + row + style.WindowPadding.x, 0.f), ImGuiCond_FirstUseEver);
}

void preset_window::render_body()
{
    take_requested();

    const bool offered = render_preset_selection();
    if(!m_visualizer.is_preset_loaded())
        return;

    if(offered)
        ImGui::SameLine();
    if(ImGui::Button(unload_label))
        m_visualizer.clear_preset();
    if(!m_visualizer.save_offered())
        return;

    ImGui::SameLine();
    if(ImGui::Button(save_label))
        m_visualizer.save_values();
}

// Drawn in place of the ordinary body, so nothing can be composed, released or written while a
// question stands. Neither answer is preselected, no key reaches either, and closing the window
// answers nothing. A release the held composition asked for names what refused, so the question says
// what is going away and why before it asks what to do with the values, and a keep that could not be
// carried out says why beneath it, each reason drawn as an argument rather than as a format string.
void preset_window::render_question()
{
    if(const std::string_view cause = m_visualizer.release_cause(); !cause.empty())
        ImGui::TextWrapped("'%.*s' refused, so what is composed cannot continue and is being unloaded.", static_cast<int>(cause.size()), cause.data());
    ImGui::TextUnformatted("What is composed differs from what the file carries.");
    if(const std::string_view refused = m_visualizer.answer_refusal(); !refused.empty())
        ImGui::TextWrapped("Keeping it failed: %.*s", static_cast<int>(refused.size()), refused.data());
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

    ImGui::SetNextItemWidth(dropdown_width(presets));
    if(render_dropdown_selection("##item", m_selected, presets))
        m_requested.reset();
    ImGui::SameLine();
    if(ImGui::Button(load_label))
        request(presets[m_selected]);

    return true;
}

}
