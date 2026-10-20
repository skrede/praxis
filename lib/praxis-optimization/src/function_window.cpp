#include "praxis/optimization/settings_keys.h"
#include "praxis/optimization/function_window.h"

#include "praxis/scene/widgets.h"

#include <imgui.h>

#include <span>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>

namespace praxis::optimization {

namespace {

constexpr std::array<const char *, 1> source_labels{"Registered"};

constexpr std::array<const char *, 2> lower_labels{"x min", "y min"};
constexpr std::array<const char *, 2> upper_labels{"x max", "y max"};

constexpr const char *unaskable_range = "The minimum and the maximum must be finite, and the minimum must lie below the maximum.";

std::size_t position_of(const std::vector<std::string> &names, const std::string &named)
{
    const auto found = std::find(names.begin(), names.end(), named);

    return found == names.end() ? 0u : static_cast<std::size_t>(found - names.begin());
}

bool askable(const std::vector<interval> &ranges)
{
    return std::ranges::all_of(ranges, [](const interval &range) { return std::isfinite(range.lower) && std::isfinite(range.upper) && range.lower < range.upper; });
}

}

std::span<const char *const> function_source_labels()
{
    return source_labels;
}

function_window::function_window(std::string name, const scenario_window_inputs &inputs)
        : imgui_window(std::move(name))
        , m_source(function_source::registered)
        , m_selected(position_of(inputs.registered, inputs.opened.selected))
        , m_typed(inputs.opened.ranges)
        , m_posted(m_typed)
        , m_registered(inputs.registered)
        , m_scenario(inputs.scenario)
{
}

std::vector<config::edit> function_window::settings_edits(const config::document &carried) const
{
    std::vector<config::edit> edits;
    if(m_selected < m_registered.size())
        edits.emplace_back(function_name_key, m_registered[m_selected]);
    for(std::size_t at = 0; at < m_posted.size(); ++at)
    {
        edits.emplace_back(range_lower_keys.at(at), config::exact_text(m_posted[at].lower));
        edits.emplace_back(range_upper_keys.at(at), config::exact_text(m_posted[at].upper));
    }

    return config::unsaved_edits(carried, edits);
}

void function_window::render()
{
    ImGui::Begin(display_name().c_str());
    static_cast<void>(scene::render_enum_selection("Source", m_source, function_source_labels()));
    if(scene::render_dropdown_selection("Function", m_selected, m_registered))
        command(m_scenario, [index = m_selected](scenario_state &state) { state.select(index); });

    bool edited = false;
    for(std::size_t at = 0; at < m_typed.size(); ++at)
        edited = render_range(at) || edited;
    if(edited)
        post_range();
    if(!askable(m_typed))
        ImGui::TextUnformatted(unaskable_range);
    ImGui::End();
}

// A range is asked for once its field is left after an edit, so the digits typed on the way to a
// value are not each solved over.
bool function_window::render_range(std::size_t coordinate)
{
    ImGui::InputDouble(lower_labels.at(coordinate), &m_typed[coordinate].lower, 0.0, 0.0, "%.10g");
    const bool lower = ImGui::IsItemDeactivatedAfterEdit();
    ImGui::InputDouble(upper_labels.at(coordinate), &m_typed[coordinate].upper, 0.0, 0.0, "%.10g");
    const bool upper = ImGui::IsItemDeactivatedAfterEdit();

    return lower || upper;
}

void function_window::post_range()
{
    if(!askable(m_typed))
        return;

    m_posted = m_typed;
    command(m_scenario, [ranges = m_posted](scenario_state &state) { state.set_range(ranges); });
}

}
