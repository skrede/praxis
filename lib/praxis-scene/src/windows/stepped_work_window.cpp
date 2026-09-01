#include "windows/stepped_work_window.h"

#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>

namespace praxis::scene {

namespace {

const std::array<const char *, 7> column_headings{"#", "Valid", "Active", "Ran", "Dropped", "Worst lateness (s)", "Overrun policy"};

// An admission that named no policy is reported as such rather than as an empty cell, because "none
// recorded" and "drop" are different facts and a blank reads as either.
const char *policy_label(const std::optional<scheduler::overrun> &policy)
{
    if(!policy.has_value())
        return "none recorded";

    return *policy == scheduler::overrun::catch_up ? "catch up" : "drop";
}

}

stepped_work_window::stepped_work_window(std::string name, visualizer &v)
        : imgui_window(std::move(name))
        , m_visualizer(v)
{
}

void stepped_work_window::render()
{
    const std::vector<stepped_work_report> reported = m_visualizer.composed_work();

    place_on_first_use();
    ImGui::Begin(display_name().c_str());
    if(reported.empty())
        ImGui::TextUnformatted("No stepped work is admitted.");
    else
        render_table(reported);
    ImGui::End();
}

void stepped_work_window::place_on_first_use() const
{
    const ImGuiViewport &viewport = *ImGui::GetMainViewport();
    const ImVec2 size(viewport.WorkSize.x * 0.45f, viewport.WorkSize.y * 0.25f);
    ImGui::SetNextWindowPos(ImVec2(viewport.WorkPos.x + viewport.WorkSize.x - size.x - 20.f, viewport.WorkPos.y + 20.f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(size, ImGuiCond_FirstUseEver);
}

void stepped_work_window::render_table(const std::vector<stepped_work_report> &reported) const
{
    if(!ImGui::BeginTable("##work", static_cast<int>(column_headings.size()), ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
        return;

    for(const char *heading : column_headings)
        ImGui::TableSetupColumn(heading);
    ImGui::TableHeadersRow();
    for(std::size_t index = 0; index < reported.size(); ++index)
        render_row(index, reported[index]);
    ImGui::EndTable();
}

void stepped_work_window::render_row(std::size_t index, const stepped_work_report &reported) const
{
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::Text("%zu", index);
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(reported.valid ? "yes" : "no");
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(reported.active ? "yes" : "no");
    ImGui::TableNextColumn();
    ImGui::Text("%llu", static_cast<unsigned long long>(reported.counters.ran));
    ImGui::TableNextColumn();
    ImGui::Text("%llu", static_cast<unsigned long long>(reported.counters.dropped));
    ImGui::TableNextColumn();
    ImGui::Text("%.6f", reported.counters.worst_lateness.count());
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(policy_label(reported.counters.policy));
}

}
