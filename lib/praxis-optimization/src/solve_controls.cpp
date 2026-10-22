#include "praxis/optimization/solve_window.h"
#include "praxis/optimization/settings_keys.h"

#include "praxis/scene/widgets.h"

#include <imgui.h>

#include <span>
#include <array>
#include <cmath>
#include <string>
#include <vector>
#include <cstddef>

namespace praxis::optimization {

namespace {

constexpr std::array<const char *, 2> seed_labels{"Seed x", "Seed y"};

constexpr std::array<const char *, 2> method_labels{"Newton", "Gradient descent"};

}

std::span<const char *const> minimizer_labels()
{
    return method_labels;
}

std::vector<config::edit> solve_window::settings_edits(const config::document &carried) const
{
    std::vector<config::edit> edits{config::edit{limit_key, std::to_string(m_limit)}, config::edit{step_key, std::to_string(m_step)}};
    for(Eigen::Index at = 0; at < m_seed.size(); ++at)
        edits.emplace_back(seed_keys.at(static_cast<std::size_t>(at)), config::exact_text(m_seed[at]));
    if(m_form == problem_form::minimum)
    {
        edits.emplace_back(method_key, method_spellings.at(static_cast<std::size_t>(m_method)));
        edits.emplace_back(step_length_key, config::exact_text(m_step_length));
    }

    return config::unsaved_edits(carried, edits);
}

void solve_window::render_controls()
{
    for(Eigen::Index at = 0; at < m_seed.size(); ++at)
        ImGui::InputDouble(seed_labels.at(static_cast<std::size_t>(at)), &m_seed[at], 0.0, 0.0, "%.10g");

    if(ImGui::InputScalar("Iteration limit", ImGuiDataType_U32, &m_limit))
        command(m_scenario, [limit = m_limit](scenario_state &state) { state.set_limit(limit); });
    if(m_form == problem_form::minimum)
        render_objective_controls();
    if(ImGui::Button("Run"))
        command(m_scenario, [seed = m_seed, limit = m_limit](scenario_state &state) { state.run(seed, limit); });
    ImGui::SameLine();
    if(ImGui::Button("Step once"))
        command(m_scenario, [seed = m_seed](scenario_state &state) { state.step_once(seed); });
    ImGui::SameLine();
    if(ImGui::Button("Reset"))
        command(m_scenario, [seed = m_seed](scenario_state &state) { state.reset(seed); });
}

void solve_window::render_objective_controls()
{
    if(scene::render_enum_selection("Method", m_method, minimizer_labels()))
        command(m_scenario, [method = m_method](scenario_state &state) { state.select_method(method); });

    ImGui::InputDouble("Step length", &m_typed_step, 0.0, 0.0, "%.10g");
    if(!ImGui::IsItemDeactivatedAfterEdit())
        return;

    if(std::isfinite(m_typed_step) && m_typed_step > 0.0)
        m_step_length = m_typed_step;
    command(m_scenario, [step_length = m_typed_step](scenario_state &state) { state.set_step_length(step_length); });
}

}
