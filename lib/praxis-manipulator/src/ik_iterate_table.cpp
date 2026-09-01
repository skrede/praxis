#include "praxis/manipulator/option_widgets.h"
#include "praxis/manipulator/ik_iterate_window.h"

#include "praxis/scene/widgets.h"

#include <imgui.h>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <algorithm>
#include <initializer_list>

namespace praxis::manipulator {

namespace {

constexpr const char *unpublished_arm   = "The arm has published nothing yet.";
constexpr const char *nothing_stepped   = "The last solve recorded no step.";
constexpr const char *start_stood_still = "This start recorded no step.";

constexpr std::initializer_list<const char *> columns{"Step", "Angular (rad)", "Linear (m)", "Change (rad)"};

// One step is one row, picked as a whole. The three numbers are the ones the step carries and no
// arithmetic over them, each in a column of its own.
bool render_step_row(const iteration_state &step, std::size_t row, bool standing)
{
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::PushID(static_cast<int>(row));
    const bool picked = ImGui::Selectable(std::to_string(step.index).c_str(), standing, ImGuiSelectableFlags_SpanAllColumns);
    ImGui::PopID();

    for(const double value : {step.angular_error, step.linear_error, step.step_norm})
    {
        ImGui::TableNextColumn();
        ImGui::Text("%.3e", value);
    }

    return picked;
}

}

void ik_iterate_window::render()
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();

    ImGui::Begin(display_name().c_str());
    if(published == nullptr)
        ImGui::TextUnformatted(unpublished_arm);
    else
        render_iterates(*published);
    ImGui::End();
}

void ik_iterate_window::render_iterates(const arm_snapshot &seen)
{
    relist(seen.iterations.size());
    render_option_cycle("Control mode", m_control_mode);
    if(m_starts.empty())
    {
        ImGui::TextUnformatted(nothing_stepped);

        return;
    }

    m_selected = std::min(m_selected, m_starts.size() - 1u);
    static_cast<void>(scene::render_dropdown_selection("Start", m_selected, m_starts));
    render_steps(seen.iterations[m_selected]);
}

void ik_iterate_window::render_steps(const std::vector<iteration_state> &steps)
{
    if(steps.empty())
    {
        ImGui::TextUnformatted(start_stood_still);

        return;
    }

    render_table(steps);
}

void ik_iterate_window::render_table(const std::vector<iteration_state> &steps)
{
    m_iterate = std::min(m_iterate, steps.size() - 1u);
    if(!ImGui::BeginTable("##steps", static_cast<int>(columns.size()), ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg))
        return;

    for(const char *column : columns)
        ImGui::TableSetupColumn(column);
    ImGui::TableHeadersRow();

    for(std::size_t row = 0; row < steps.size(); ++row)
        if(render_step_row(steps[row], row, m_iterate == row))
        {
            m_iterate = row;
            step_to(steps[row].joint_positions);
        }

    ImGui::EndTable();
}

}
