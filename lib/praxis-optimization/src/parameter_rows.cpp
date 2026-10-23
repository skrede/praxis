#include "typed_function_panel.h"

#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/scenario_snapshot.h"

#include <imgui.h>

#include <cmath>
#include <memory>
#include <vector>
#include <utility>
#include <optional>
#include <algorithm>

namespace praxis::optimization {

namespace {

bool askable(const interval &range)
{
    return std::isfinite(range.lower) && std::isfinite(range.upper) && range.lower < range.upper;
}

}

parameter_rows::parameter_rows(scenario_reader seen, std::weak_ptr<owned_scenario> scenario)
        : m_seen(std::move(seen))
        , m_rows()
        , m_wanted()
        , m_in_flight()
        , m_scenario(std::move(scenario))
        , m_last()
{
}

void parameter_rows::render()
{
    catch_up(m_seen.read());
    for(parameter_row &row : m_rows)
        render_row(row);
    post_wanted();
}

std::vector<typed_parameter> parameter_rows::held() const
{
    std::vector<typed_parameter> shown;
    for(const parameter_row &row : m_rows)
        shown.push_back(row.shown);

    return shown;
}

void parameter_rows::catch_up(const std::shared_ptr<const scenario_snapshot> &shown)
{
    if(shown == m_last || shown == nullptr)
        return;

    m_last                                        = shown;
    const std::vector<typed_parameter> &published = shown->parameters;
    answer_requests(published);

    std::vector<parameter_row> rows;
    for(const typed_parameter &parameter : published)
    {
        const auto kept = std::ranges::find(m_rows, parameter.name, [](const parameter_row &row) { return row.shown.name; });
        if(kept != m_rows.end() && (kept->edited || waiting(parameter.name)))
            rows.push_back(*kept);
        else
            rows.push_back(parameter_row{false, parameter.range, parameter});
    }
    m_rows = std::move(rows);
}

// A value stays in flight until a publication carries it, or no longer carries its letter, and a value
// waiting for a letter a publication no longer carries is dropped, since the state would refuse it.
void parameter_rows::answer_requests(const std::vector<typed_parameter> &published)
{
    const auto carried = [&published](char name) { return std::ranges::find(published, name, &typed_parameter::name); };
    if(m_in_flight && (carried(m_in_flight->name) == published.end() || carried(m_in_flight->name)->value == m_in_flight->value))
        m_in_flight.reset();
    std::erase_if(m_wanted, [&carried, &published](const asked_value &one) { return carried(one.name) == published.end(); });
}

void parameter_rows::render_row(parameter_row &row)
{
    ImGui::PushID(static_cast<int>(row.shown.name));
    const char letter[2] = {row.shown.name, '\0'};
    const double before  = row.shown.value;
    ImGui::SliderScalar(letter, ImGuiDataType_Double, &row.shown.value, &row.shown.range.lower, &row.shown.range.upper, "%.6g", ImGuiSliderFlags_AlwaysClamp);
    const bool sliding = ImGui::IsItemActive();
    if(ImGui::IsItemEdited())
        want(row, before);

    const bool typing = render_range(row);
    row.edited        = sliding || typing;
    ImGui::PopID();
}

// Answers whether either field is being typed into.
bool parameter_rows::render_range(parameter_row &row)
{
    const float half = 0.5f * (ImGui::CalcItemWidth() - ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::SetNextItemWidth(half);
    ImGui::InputDouble("min", &row.typed.lower, 0.0, 0.0, "%.10g");
    const bool lower = ImGui::IsItemDeactivatedAfterEdit();
    bool typing      = ImGui::IsItemActive();
    ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::SetNextItemWidth(half);
    ImGui::InputDouble("max", &row.typed.upper, 0.0, 0.0, "%.10g");
    const bool upper = ImGui::IsItemDeactivatedAfterEdit();
    typing           = typing || ImGui::IsItemActive();

    if((lower || upper) && askable(row.typed))
        command(m_scenario, [name = row.shown.name, range = row.typed](scenario_state &state) { state.set_parameter_range(name, range); });
    if(!askable(row.typed))
        ImGui::TextUnformatted(unaskable_range);

    return typing;
}

// A typed value is held to the slider's range, and one that is no number is put back rather than asked for.
void parameter_rows::want(parameter_row &row, double before)
{
    if(!std::isfinite(row.shown.value))
    {
        row.shown.value = before;
        return;
    }

    const auto queued = std::ranges::find(m_wanted, row.shown.name, &asked_value::name);
    if(queued != m_wanted.end())
        queued->value = row.shown.value;
    else
        m_wanted.push_back(asked_value{row.shown.name, row.shown.value});
}

// One value is on the work strand at a time; the latest wanted for each letter follows it, in the order
// the letters were first wanted.
void parameter_rows::post_wanted()
{
    if(m_wanted.empty() || m_in_flight)
        return;

    m_in_flight = m_wanted.front();
    m_wanted.erase(m_wanted.begin());
    command(m_scenario, [asked = *m_in_flight](scenario_state &state) { state.set_parameter(asked.name, asked.value); });
}

bool parameter_rows::waiting(char name) const
{
    return std::ranges::find(m_wanted, name, &asked_value::name) != m_wanted.end() || (m_in_flight && m_in_flight->name == name);
}

}
