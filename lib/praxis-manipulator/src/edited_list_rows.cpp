#include "joint_naming.h"
#include "edited_list_layout.h"

#include "praxis/manipulator/edited_list_rows.h"

#include "praxis/rigid_motion/angles.h"

#include <imgui.h>

#include <Eigen/Core>

#include <span>
#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>

namespace praxis::manipulator {

namespace {

// One value of a row is drawn this wide, in pixels, whichever kind of row it belongs to.
constexpr float value_width = 72.f;

constexpr std::array<const char *, 6> pose_columns{"X", "Y", "Z", "A", "B", "C"};

constexpr const char *no_joints = "asked for a row on an arm of no joints";
constexpr const char *no_tool   = "asked for a row from a publication carrying no tool pose";

}

std::string row_name(std::size_t row)
{
    return "w" + std::to_string(row + 1u);
}

float row_value_pitch()
{
    return value_width + ImGui::GetStyle().ItemInnerSpacing.x;
}

// An input holding several values splits the width it is given evenly and sets the inner spacing
// between them, so a run asked for at this width draws each of its values at `value_width`.
float row_values_width(std::size_t values)
{
    return values == 0u ? 0.f : static_cast<float>(values) * row_value_pitch() - ImGui::GetStyle().ItemInnerSpacing.x;
}

float row_values_offset(std::size_t rows)
{
    const std::string widest = row_name(rows == 0u ? 0u : rows - 1u);

    return ImGui::GetCursorPosX() + 2.f * ImGui::GetFrameHeight() + 3.f * ImGui::GetStyle().ItemSpacing.x + ImGui::CalcTextSize(widest.c_str()).x;
}

bool render_ordering_arrow(const char *id, ImGuiDir toward, bool at_the_end)
{
    ImGui::BeginDisabled(at_the_end);
    const bool pressed = ImGui::ArrowButton(id, toward);
    ImGui::EndDisabled();

    return pressed;
}

void render_row_columns(std::span<const std::string> named, float from)
{
    for(std::size_t column = 0; column < named.size(); ++column)
    {
        if(column == 0u)
            ImGui::SetCursorPosX(from);
        else
            ImGui::SameLine(from + static_cast<float>(column) * row_value_pitch());

        ImGui::TextUnformatted(named[column].c_str());
    }
}

list_row_traits<joint_vector>::shown_row list_row_traits<joint_vector>::shown(const joint_vector &row, const rigid_motion::frame_ops &)
{
    return (row * degrees_per_radian).cast<float>();
}

joint_vector list_row_traits<joint_vector>::taken(const shown_row &row, const rigid_motion::frame_ops &)
{
    return row.cast<double>() * radians_per_degree;
}

std::optional<std::string> list_row_traits<joint_vector>::fault_of(const joint_vector &row, std::size_t joints)
{
    if(joints != 0u && row.size() == static_cast<Eigen::Index>(joints))
        return std::nullopt;

    return "a row of " + std::to_string(row.size()) + " joint values for an arm of " + std::to_string(joints) + " joints";
}

std::optional<std::string> list_row_traits<joint_vector>::captured(shown_row &into, const arm_snapshot &seen, const rigid_motion::frame_ops &frames)
{
    if(seen.joints.size() == 0)
        return no_joints;

    into = shown(seen.joints, frames);

    return std::nullopt;
}

std::vector<std::string> list_row_traits<joint_vector>::column_labels(std::size_t joints)
{
    return named_joints(joints);
}

bool list_row_traits<joint_vector>::render(shown_row &row)
{
    ImGui::SetNextItemWidth(row_values_width(static_cast<std::size_t>(row.size())));

    return ImGui::InputScalarN("##joints", ImGuiDataType_Float, row.data(), static_cast<int>(row.size()), nullptr, nullptr, "%.1f");
}

list_row_traits<edited_pose>::shown_row list_row_traits<edited_pose>::shown(const edited_pose &row, const rigid_motion::frame_ops &)
{
    return row;
}

edited_pose list_row_traits<edited_pose>::taken(const shown_row &row, const rigid_motion::frame_ops &)
{
    return row;
}

std::optional<std::string> list_row_traits<edited_pose>::fault_of(const edited_pose &, std::size_t)
{
    return std::nullopt;
}

std::optional<std::string> list_row_traits<edited_pose>::captured(shown_row &into, const arm_snapshot &seen, const rigid_motion::frame_ops &frames)
{
    if(!seed_from(into, seen, frames))
        return no_tool;

    return std::nullopt;
}

std::vector<std::string> list_row_traits<edited_pose>::column_labels(std::size_t)
{
    return std::vector<std::string>(pose_columns.begin(), pose_columns.end());
}

// The two triples stand on the pitch every other row's values stand on, so the six of them read as
// one run of columns rather than as two groups placed beside each other.
bool list_row_traits<edited_pose>::render(shown_row &row)
{
    ImGui::SetNextItemWidth(row_values_width(3u));
    const bool moved = ImGui::InputScalarN("##position", ImGuiDataType_Float, row.position.data(), 3, nullptr, nullptr, "%.3f");
    ImGui::SameLine(0.f, ImGui::GetStyle().ItemInnerSpacing.x);
    ImGui::SetNextItemWidth(row_values_width(3u));

    return ImGui::InputScalarN("##angles", ImGuiDataType_Float, row.euler_degrees.data(), 3, nullptr, nullptr, "%.1f") || moved;
}

}
