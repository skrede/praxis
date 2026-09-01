#include "praxis/manipulator/ik_seed_window.h"
#include "praxis/manipulator/kinematics_configuration.h"

#include "praxis/extension/held_handle.h"

#include "praxis/rigid_motion/angles.h"

#include <imgui.h>

#include <spdlog/spdlog.h>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <numbers>
#include <utility>
#include <algorithm>

namespace praxis::manipulator {

namespace {

constexpr std::size_t opening_starts = 8;

constexpr const char *unpublished_arm = "The arm has published nothing yet.";

// Van der Corput's radical inverse of `index` in `base`, which is one dimension of the Halton
// sequence: Halton, Numerische Mathematik 2 (1960), 84-90.
double radical_inverse(std::size_t base, std::size_t index)
{
    double inverted = 0.0;
    double fraction = 1.0 / static_cast<double>(base);
    for(std::size_t left = index; left != 0u; left /= base)
    {
        inverted += fraction * static_cast<double>(left % base);
        fraction /= static_cast<double>(base);
    }

    return inverted;
}

std::size_t prime_at(std::size_t which)
{
    std::size_t found     = 0;
    std::size_t candidate = 1;
    while(found <= which)
    {
        ++candidate;
        bool prime = true;
        for(std::size_t divisor = 2; divisor * divisor <= candidate; ++divisor)
            prime = prime && candidate % divisor != 0u;
        found += prime ? 1u : 0u;
    }

    return candidate;
}

joint_vector spread_start(std::size_t joints, std::size_t start)
{
    constexpr double turn = 2.0 * std::numbers::pi;

    joint_vector seed = joint_vector::Zero(static_cast<Eigen::Index>(joints));
    for(std::size_t joint = 0; start != 0u && joint < joints; ++joint)
        seed[static_cast<Eigen::Index>(joint)] = (radical_inverse(prime_at(joint), start) - 0.5) * turn;

    return seed;
}

}

ik_seed_window::settings::settings(std::vector<joint_vector> chosen)
        : seeds(std::move(chosen))
{
}

std::vector<joint_vector> ik_seed_window::opening_seeds(std::size_t joints)
{
    std::vector<joint_vector> seeds;
    for(std::size_t start = 0; joints != 0u && start < opening_starts; ++start)
        seeds.push_back(spread_start(joints, start));

    return seeds;
}

ik_seed_window::ik_seed_window(std::string name, arm_reader seen)
        : ik_seed_window(std::move(name), std::move(seen), settings{})
{
}

ik_seed_window::ik_seed_window(std::string name, arm_reader seen, const settings &state, std::string at)
        : imgui_window(std::move(name))
        , m_seen(seen)
        , m_settings_at(std::move(at))
{
    const std::shared_ptr<const arm_snapshot> share = seen.read();
    const std::size_t joints                        = static_cast<std::size_t>(held(share, "the seed list window", "published arm state").joints.size());
    const std::vector<joint_vector> taken           = state.seeds.empty() ? opening_seeds(joints) : state.seeds;

    for(std::size_t row = 0; row < taken.size(); ++row)
        accept(taken[row], row, joints);
}

void ik_seed_window::accept(const joint_vector &seed, std::size_t row, std::size_t joints)
{
    if(joints == 0u || seed.size() != static_cast<Eigen::Index>(joints))
    {
        spdlog::error("praxis: 'manipulator.ik_seed_window' was given a start of {} joint values at row {} for an arm of {} joints, so it was not taken into the list and the "
                      "starts beside it still stand",
                      seed.size(), row + 1u, joints);

        return;
    }

    m_degrees.push_back((seed * degrees_per_radian).cast<float>());
}

ik_seed_window::settings ik_seed_window::state() const
{
    std::vector<joint_vector> seeds;
    for(const Eigen::VectorXf &row : m_degrees)
        seeds.push_back(row.cast<double>() * radians_per_degree);

    return settings{std::move(seeds)};
}

std::vector<config::edit> ik_seed_window::settings_edits(const config::document &carried) const
{
    return config::unsaved_edits(carried, write_ik_seeds(carried, state(), m_settings_at));
}

void ik_seed_window::render()
{
    const std::shared_ptr<const arm_snapshot> published = m_seen.read();

    ImGui::Begin(display_name().c_str());
    if(published == nullptr)
        ImGui::TextUnformatted(unpublished_arm);
    else
        render_seeds(static_cast<std::size_t>(published->joints.size()));
    ImGui::End();
}

void ik_seed_window::render_seeds(std::size_t joints)
{
    std::size_t named = m_degrees.size();
    row_edit asked    = row_edit::none;
    for(std::size_t row = 0; row < m_degrees.size(); ++row)
    {
        const row_edit taken = render_row(row);
        named                = taken == row_edit::none ? named : row;
        asked                = taken == row_edit::none ? asked : taken;
    }

    if(ImGui::Button("Add"))
        append(joints);
    if(asked == row_edit::remove)
        m_degrees.erase(m_degrees.begin() + static_cast<std::ptrdiff_t>(named));
    if(asked == row_edit::raise && named != 0u)
        std::swap(m_degrees[named], m_degrees[named - 1u]);
}

ik_seed_window::row_edit ik_seed_window::render_row(std::size_t row)
{
    ImGui::PushID(static_cast<int>(row));
    const bool removing = ImGui::Button("Remove");
    ImGui::SameLine();
    const bool raising = ImGui::Button("Up");
    ImGui::SameLine();
    ImGui::Text("s%zu", row + 1u);
    ImGui::SameLine();
    ImGui::InputScalarN("##start", ImGuiDataType_Float, m_degrees[row].data(), static_cast<int>(m_degrees[row].size()), nullptr, nullptr, "%.1f");
    ImGui::PopID();

    return removing ? row_edit::remove : raising ? row_edit::raise : row_edit::none;
}

void ik_seed_window::append(std::size_t joints)
{
    if(joints == 0u)
    {
        spdlog::error("praxis: 'manipulator.ik_seed_window' was asked for a start on an arm of no joints, so none was added and the list is as it was");

        return;
    }

    m_degrees.push_back(Eigen::VectorXf::Zero(static_cast<Eigen::Index>(joints)));
}

}
