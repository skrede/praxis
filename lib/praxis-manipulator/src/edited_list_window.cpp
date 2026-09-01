#include "edited_list_layout.h"

#include "praxis/manipulator/edited_list_window.h"

#include "praxis/extension/held_handle.h"

#include <imgui.h>

#include <spdlog/spdlog.h>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>
#include <functional>

namespace praxis::manipulator {

namespace {

constexpr const char *unpublished_arm = "The arm has published nothing yet.";

constexpr const char *list_holder = "the edited list window";

}

template<typename Row>
edited_list_window<Row>::settings::settings(std::vector<Row> chosen)
        : rows(std::move(chosen))
{
}

template<typename Row>
std::vector<Row> edited_list_window<Row>::opening_rows(std::size_t joints)
{
    return row_traits::opening_rows(joints);
}

template<typename Row>
edited_list_window<Row>::edited_list_window(std::string name, arm_reader seen, const rigid_motion::frame_ops &injected, std::function<void()> edited)
        : edited_list_window(std::move(name), std::move(seen), injected, settings{}, std::string(), std::move(edited))
{
}

template<typename Row>
edited_list_window<Row>::edited_list_window(std::string name, arm_reader seen, const rigid_motion::frame_ops &injected, const settings &state, std::string at,
                                            std::function<void()> edited)
        : imgui_window(std::move(name))
        , m_seen(seen)
        , m_settings_at(std::move(at))
        , m_frame(injected)
        , m_edited_cb(std::move(edited))
{
    const std::shared_ptr<const arm_snapshot> share = seen.read();
    const std::size_t joints                        = static_cast<std::size_t>(held(share, list_holder, "published arm state").joints.size());
    const std::vector<Row> taken                    = state.rows.empty() ? opening_rows(joints) : state.rows;

    for(std::size_t row = 0; row < taken.size(); ++row)
        accept(taken[row], row, joints);
}

template<typename Row>
void edited_list_window<Row>::accept(const Row &given, std::size_t row, std::size_t joints)
{
    const std::optional<std::string> fault = row_traits::fault_of(given, joints);
    if(fault)
    {
        spdlog::error("praxis: 'manipulator.edited_list_window' was given {} at row {}, so it was not taken into the list and the rows beside it still stand", *fault, row + 1u);

        return;
    }

    m_shown.push_back(row_traits::shown(given, m_frame));
}

template<typename Row>
typename edited_list_window<Row>::settings edited_list_window<Row>::state() const
{
    std::vector<Row> rows;
    for(const shown_row &row : m_shown)
        rows.push_back(row_traits::taken(row, m_frame));

    return settings{std::move(rows)};
}

template<typename Row>
std::vector<config::edit> edited_list_window<Row>::settings_edits(const config::document &carried) const
{
    return config::unsaved_edits(carried, row_traits::written(carried, state().rows, m_settings_at));
}

template<typename Row>
void edited_list_window<Row>::render()
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();

    ImGui::Begin(display_name().c_str());
    if(published == nullptr)
        ImGui::TextUnformatted(unpublished_arm);
    else
        render_rows(*published);
    ImGui::End();
}

template<typename Row>
void edited_list_window<Row>::render_rows(const arm_snapshot &seen)
{
    const float values_at = row_values_offset(m_shown.size());
    if(!m_shown.empty())
        render_row_columns(row_traits::column_labels(static_cast<std::size_t>(seen.joints.size())), values_at);

    std::size_t named = m_shown.size();
    row_edit asked    = row_edit::none;
    bool changed      = false;
    for(std::size_t row = 0; row < m_shown.size(); ++row)
    {
        const row_asked taken = render_row(row, values_at);
        named                 = taken.edit == row_edit::none ? named : row;
        asked                 = taken.edit == row_edit::none ? asked : taken.edit;
        changed               = taken.typed || changed;
    }

    if(ImGui::Button("Add"))
        changed = append(seen) || changed;

    changed = applied(asked, named) || changed;
    if(changed && m_edited_cb)
        m_edited_cb();
}

template<typename Row>
typename edited_list_window<Row>::row_asked edited_list_window<Row>::render_row(std::size_t row, float values_at)
{
    ImGui::PushID(static_cast<int>(row));
    const bool raising = render_ordering_arrow("##raise", ImGuiDir_Up, row == 0u);
    ImGui::SameLine();
    const bool lowering = render_ordering_arrow("##lower", ImGuiDir_Down, row + 1u == m_shown.size());
    ImGui::SameLine();
    ImGui::TextUnformatted(row_name(row).c_str());
    ImGui::SameLine(values_at);
    const bool typed = row_traits::render(m_shown[row]);
    ImGui::SameLine();
    const bool removing = ImGui::SmallButton("x");
    ImGui::PopID();

    const row_edit asked = removing ? row_edit::remove : raising ? row_edit::raise : lowering ? row_edit::lower : row_edit::none;

    return row_asked{asked, typed};
}

template<typename Row>
bool edited_list_window<Row>::applied(row_edit asked, std::size_t named)
{
    if(asked == row_edit::remove)
    {
        m_shown.erase(m_shown.begin() + static_cast<std::ptrdiff_t>(named));

        return true;
    }
    if(asked == row_edit::raise && named != 0u)
    {
        std::swap(m_shown[named], m_shown[named - 1u]);

        return true;
    }
    if(asked == row_edit::lower && named + 1u < m_shown.size())
    {
        std::swap(m_shown[named], m_shown[named + 1u]);

        return true;
    }

    return false;
}

template<typename Row>
bool edited_list_window<Row>::append(const arm_snapshot &seen)
{
    shown_row taken{};
    const std::optional<std::string> fault = row_traits::captured(taken, seen, m_frame);
    if(fault)
    {
        spdlog::error("praxis: 'manipulator.edited_list_window' was {}, so no row was added and the list is as it was", *fault);

        return false;
    }

    m_shown.push_back(std::move(taken));

    return true;
}

template class edited_list_window<joint_vector>;
template class edited_list_window<edited_pose>;

}
