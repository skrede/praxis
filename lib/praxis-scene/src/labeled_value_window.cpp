#include "praxis/scene/labeled_value_window.h"

#include <imgui.h>

#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>
#include <functional>

namespace praxis::scene {

namespace {

using value_row  = std::vector<labeled_value>;
using value_rows = std::vector<value_row>;

// The half-open run of rows one alignment spans, and the column count it is sized to. A run whose
// end meets its start stands for a row that is not drawn as aligned columns.
struct aligned_run
{
    std::size_t first;
    std::size_t past_end;
    int columns;
};

void render_entries(const value_row &row)
{
    for(std::size_t entry = 0; entry < row.size(); ++entry)
    {
        if(entry > 0)
            ImGui::SameLine();

        const labeled_value &cell = row[entry];
        if(cell.label.empty() && cell.stated.empty())
            // The digits a labeled cell prints, so one value reads the same drawn either way.
            ImGui::Text("%.3f", cell.value);
        else if(cell.label.empty())
            ImGui::TextUnformatted(cell.stated.c_str());
        else if(cell.stated.empty())
            ImGui::Value(cell.label.c_str(), cell.value);
        else
            ImGui::Text("%s: %s", cell.label.c_str(), cell.stated.c_str());
    }
}

bool unlabeled(const value_row &row)
{
    const auto carries_label = [](const labeled_value &cell) { return !cell.label.empty(); };

    return !row.empty() && std::none_of(row.begin(), row.end(), carries_label);
}

aligned_run run_from(const value_rows &rows, std::size_t first)
{
    aligned_run found{first, first, 0};
    while(found.past_end < rows.size() && unlabeled(rows[found.past_end]))
    {
        found.columns = std::max(found.columns, static_cast<int>(rows[found.past_end].size()));
        ++found.past_end;
    }

    return found;
}

void render_aligned(const std::string &identity, const value_rows &rows, const aligned_run &run)
{
    if(!ImGui::BeginTable(identity.c_str(), run.columns, ImGuiTableFlags_SizingFixedFit))
        return;

    for(std::size_t index = run.first; index < run.past_end; ++index)
    {
        ImGui::TableNextRow();
        for(const labeled_value &cell : rows[index])
        {
            ImGui::TableNextColumn();
            if(!cell.stated.empty())
                ImGui::TextUnformatted(cell.stated.c_str());
            else
                // The digits a labeled cell prints, so one value reads the same drawn either way.
                ImGui::Text("%.3f", cell.value);
        }
    }

    ImGui::EndTable();
}

void render_rows(const std::string &name, const value_rows &rows)
{
    std::size_t index   = 0;
    std::size_t aligned = 0;
    while(index < rows.size())
    {
        const aligned_run run = run_from(rows, index);
        if(run.past_end == run.first)
        {
            render_entries(rows[index++]);
            continue;
        }

        render_aligned(name + "##aligned" + std::to_string(aligned++), rows, run);
        index = run.past_end;
    }
}

}

labeled_value_window::labeled_value_window(std::string name, std::function<void()> controls, readout_source source)
        : imgui_window(std::move(name))
        , m_source(std::move(source))
        , m_controls(std::move(controls))
{
}

void labeled_value_window::render()
{
    ImGui::Begin(display_name().c_str());

    if(m_controls)
        m_controls();

    const readout shown = m_source ? m_source() : readout{};
    if(!shown.message.empty())
        ImGui::TextUnformatted(shown.message.c_str());
    else
        render_rows(display_name(), shown.rows);

    ImGui::End();
}

}
