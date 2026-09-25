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
constexpr const char *stated_surplus    = "Surplus";
constexpr const char *stated_unbounded  = "unbounded";
constexpr const char *stated_kinds      = "the kinds differ";
constexpr const char *stated_unsupplied = "not supplied";

constexpr const char *turned_label = "Turned from the described chain (rad)";
constexpr const char *moved_label  = "Moved from the described chain (m)";

constexpr std::initializer_list<const char *> columns{"Line", "Rotation (rad)", "Distance (m)", "Defect"};

constexpr std::size_t whole_chain_lines_count = 2u;

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

// The columns measure a transform and a surplus fills none of them, so the two counts are said in
// words on a line that carries no term at all.
value_row surplus_row(std::size_t supplied, std::size_t joints)
{
    return value_row{stating(stated_surplus), stating(std::to_string(supplied) + " screws supplied against a chain of " + std::to_string(joints))};
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

// Each whole-chain line is a label and one number, at the six decimals this window has printed them
// at since it carried nothing else. A line stating something carries that where the number stood.
void render_whole_chain_lines(const scene::readout &shown)
{
    for(std::size_t line = 0u; line < whole_chain_lines_count && line < shown.rows.size(); ++line)
    {
        const scene::labeled_value &only = shown.rows[line].front();
        if(only.stated.empty())
            ImGui::Text("%s %.6f", only.label.c_str(), static_cast<double>(only.value));
        else
            ImGui::Text("%s %s", only.label.c_str(), only.stated.c_str());
    }
}

void render_appended_table(const scene::readout &shown)
{
    if(shown.rows.size() <= whole_chain_lines_count)
        return;
    if(!ImGui::BeginTable("##chain", static_cast<int>(columns.size()), ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg))
        return;

    for(const char *column : columns)
        ImGui::TableSetupColumn(column);
    ImGui::TableHeadersRow();

    for(std::size_t line = whole_chain_lines_count; line < shown.rows.size(); ++line)
        render_line(shown.rows[line]);

    ImGui::EndTable();
}

}

scene::readout whole_chain_lines(const evaluation::residual &apart)
{
    const scene::labeled_value turned{static_cast<float>(apart.magnitude), turned_label};
    const scene::labeled_value moved{static_cast<float>(apart.linear_error_metres), moved_label};

    return scene::readout{{}, {{turned}, {moved}}};
}

scene::readout whole_chain_without_pose(const char *said)
{
    const scene::labeled_value turned{0.0f, turned_label, said};
    const scene::labeled_value moved{0.0f, moved_label, said};

    return scene::readout{{}, {{turned}, {moved}}};
}

scene::readout screw_modeling_reading(const screw_chain_difference &apart, scene::readout carried)
{
    carried.rows.reserve(carried.rows.size() + apart.joints.size() + 2u);
    carried.rows.push_back(home_row(apart.home));
    for(std::size_t joint = 0u; joint < apart.joints.size(); ++joint)
        carried.rows.push_back(joint_row(apart.joints[joint], joint));
    if(apart.supplied > apart.joints.size())
        carried.rows.push_back(surplus_row(apart.supplied, apart.joints.size()));

    return carried;
}

void render_screw_modeling_reading(const scene::readout &shown)
{
    if(!shown.message.empty())
        return ImGui::TextUnformatted(shown.message.c_str());

    render_whole_chain_lines(shown);
    render_appended_table(shown);
}

}
