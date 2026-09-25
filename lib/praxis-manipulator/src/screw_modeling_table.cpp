#include "screw_modeling_table.h"

#include <imgui.h>

#include <cmath>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <initializer_list>

namespace praxis::manipulator {

namespace {

using value_row = std::vector<scene::labeled_value>;

constexpr const char *stated_home       = "Home";
constexpr const char *stated_absent     = "absent";
constexpr const char *stated_unbounded  = "unbounded";
constexpr const char *stated_kinds      = "the kinds differ";
constexpr const char *stated_unsupplied = "not supplied";

constexpr std::initializer_list<const char *> columns{"Line", "Rotation (rad)", "Distance (m)", "Defect"};

scene::labeled_value stating(std::string said)
{
    return scene::labeled_value{0.0f, std::string(), std::move(said)};
}

// A value that is not finite reaching a text field is the one thing no cell may carry, so it is
// stated instead of printed.
scene::labeled_value cell(double value)
{
    if(!std::isfinite(value))
        return stating(stated_unbounded);

    return scene::labeled_value{static_cast<float>(value), std::string(), std::string()};
}

scene::labeled_value term(const std::optional<double> &value)
{
    return value ? cell(*value) : stating(stated_absent);
}

value_row home_row(const chain_home_difference &home)
{
    return value_row{stating(stated_home), cell(home.turned_radians), cell(home.moved_metres), cell(home.rigidity)};
}

// The joint is named from one, the way the window's own list of them counts. The moment stands in
// the metres column and the length in the defect column, so every column carries one unit whichever
// line is read across it.
value_row joint_row(const chain_joint_difference &line, std::size_t joint)
{
    scene::labeled_value named = stating("Joint " + std::to_string(joint + 1u));
    if(line.read == chain_joint_reading::not_supplied)
        return value_row{std::move(named), stating(stated_unsupplied)};
    if(line.read == chain_joint_reading::kinds_differed)
        return value_row{std::move(named), stating(stated_kinds)};

    return value_row{std::move(named), term(line.direction_radians), term(line.moment_metres), term(line.length)};
}

// A statement stands in place of the cell's number, which is how a line carrying no number for a
// term says so rather than standing a zero there.
void render_line(const value_row &line)
{
    ImGui::TableNextRow();
    for(const scene::labeled_value &shown : line)
    {
        ImGui::TableNextColumn();
        if(!shown.stated.empty())
            ImGui::TextUnformatted(shown.stated.c_str());
        else
            ImGui::Text("%.3e", static_cast<double>(shown.value));
    }
}

}

scene::readout screw_modeling_reading(const screw_chain_difference &apart, scene::readout carried)
{
    carried.rows.reserve(carried.rows.size() + apart.joints.size() + 1u);
    carried.rows.push_back(home_row(apart.home));
    for(std::size_t joint = 0u; joint < apart.joints.size(); ++joint)
        carried.rows.push_back(joint_row(apart.joints[joint], joint));

    return carried;
}

void render_screw_modeling_table(const scene::readout &shown, std::size_t whole_chain_rows)
{
    if(shown.rows.size() <= whole_chain_rows)
        return;
    if(!ImGui::BeginTable("##chain", static_cast<int>(columns.size()), ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg))
        return;

    for(const char *column : columns)
        ImGui::TableSetupColumn(column);
    ImGui::TableHeadersRow();

    for(std::size_t row = whole_chain_rows; row < shown.rows.size(); ++row)
        render_line(shown.rows[row]);

    ImGui::EndTable();
}

}
