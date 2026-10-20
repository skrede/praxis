#include "scenario_answers.h"

#include "praxis/optimization/solve_window.h"
#include "praxis/optimization/settings_keys.h"

#include "praxis/extension/refusal.h"

#include <imgui.h>

#include <array>
#include <format>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <initializer_list>

namespace praxis::optimization {

namespace {

constexpr std::array<const char *, 2> seed_labels{"Seed x", "Seed y"};

constexpr std::initializer_list<const char *> columns{"k", "x_k", "e_k"};

std::string coordinates(const Eigen::VectorXd &x)
{
    std::string written;
    for(Eigen::Index at = 0; at < x.size(); ++at)
        written += std::format("{}{:.12g}", at == 0 ? "" : ", ", x[at]);

    return written;
}

std::string errors(const expected<Eigen::VectorXd, refusal> &e)
{
    if(!e)
        return std::string(refusal_text(e.error()));

    std::string written;
    for(Eigen::Index at = 0; at < e->size(); ++at)
        written += std::format("{}{:.3e}", at == 0 ? "" : ", ", (*e)[at]);

    return written;
}

void render_row(const iterate_row &shown)
{
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::Text("%zu", shown.k);
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(shown.x.c_str());
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(shown.e.c_str());
}

}

std::vector<iterate_row> iterate_rows(const scenario_run &run)
{
    std::vector<iterate_row> rows;
    for(std::size_t k = 0; k < run.iterates.size(); ++k)
        rows.push_back(iterate_row{k, coordinates(run.iterates[k]), k < run.errors.size() ? errors(run.errors[k]) : std::string()});

    return rows;
}

std::string run_status(const scenario_snapshot &shown)
{
    const std::uint32_t taken = steps_taken(shown.run);
    if(!shown.run.refused)
        return std::format("Converged. Steps taken: {}.", taken);
    if(shown.run.answer)
        return std::format("Answered where the problem has not converged. Steps taken: {}.", taken);
    if(shown.stepped && *shown.run.refused == refusal::no_solution)
        return std::format("Steps taken: {}.", taken);

    return std::format("Refused: {}. Steps taken: {}.", refusal_text(*shown.run.refused), taken);
}

solve_window::solve_window(std::string name, const scenario_window_inputs &inputs)
        : imgui_window(std::move(name))
        , m_step(inputs.opened.step)
        , m_limit(inputs.opened.limit)
        , m_seed(inputs.opened.seed)
        , m_seen(inputs.seen)
        , m_rows()
        , m_scenario(inputs.scenario)
        , m_last()
{
}

std::vector<config::edit> solve_window::settings_edits(const config::document &carried) const
{
    std::vector<config::edit> edits{config::edit{limit_key, std::to_string(m_limit)}, config::edit{step_key, std::to_string(m_step)}};
    for(Eigen::Index at = 0; at < m_seed.size(); ++at)
        edits.emplace_back(seed_keys.at(static_cast<std::size_t>(at)), config::exact_text(m_seed[at]));

    return config::unsaved_edits(carried, edits);
}

void solve_window::render()
{
    const std::shared_ptr<const scenario_snapshot> shown = m_seen.read();

    ImGui::Begin(display_name().c_str());
    render_controls();
    if(shown == nullptr)
        ImGui::TextUnformatted(nothing_published);
    else
        render_run(shown);
    ImGui::End();
}

void solve_window::render_controls()
{
    for(Eigen::Index at = 0; at < m_seed.size(); ++at)
        ImGui::InputDouble(seed_labels.at(static_cast<std::size_t>(at)), &m_seed[at], 0.0, 0.0, "%.10g");

    if(ImGui::InputScalar("Iteration limit", ImGuiDataType_U32, &m_limit))
        command(m_scenario, [limit = m_limit](scenario_state &state) { state.set_limit(limit); });
    if(ImGui::Button("Run"))
        command(m_scenario, [seed = m_seed, limit = m_limit](scenario_state &state) { state.run(seed, limit); });
    ImGui::SameLine();
    if(ImGui::Button("Step once"))
        command(m_scenario, [seed = m_seed](scenario_state &state) { state.step_once(seed); });
}

// A publication not seen before is what the slider and the table are brought to, so a selection
// made elsewhere is the one shown; so is a seed a run was started from elsewhere, while a seed typed
// and not yet run stays in its field through any publication that keeps the last run's seed.
void solve_window::render_run(const std::shared_ptr<const scenario_snapshot> &shown)
{
    if(shown != m_last)
    {
        if(m_last == nullptr || m_last->run.seed != shown->run.seed)
            m_seed = shown->run.seed;
        m_last = shown;
        m_step = shown->step;
        m_rows = iterate_rows(shown->run);
    }

    render_slider(shown->run);
    ImGui::TextUnformatted(run_status(*shown).c_str());
    render_table();
}

void solve_window::render_slider(const scenario_run &run)
{
    const std::uint32_t taken = steps_taken(run);
    if(taken == 0u)
        return;

    const std::uint32_t first = 0u;
    const std::uint32_t last  = taken - 1u;
    if(ImGui::SliderScalar("Step", ImGuiDataType_U32, &m_step, &first, &last))
        command(m_scenario, [step = m_step](scenario_state &state) { state.select_step(step); });
}

void solve_window::render_table() const
{
    if(!ImGui::BeginTable("##iterates", static_cast<int>(columns.size()), ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit))
        return;

    ImGui::TableSetupScrollFreeze(0, 1);
    for(const char *column : columns)
        ImGui::TableSetupColumn(column);
    ImGui::TableHeadersRow();

    ImGuiListClipper clipper;
    clipper.Begin(static_cast<int>(m_rows.size()));
    while(clipper.Step())
        for(int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row)
            render_row(m_rows[static_cast<std::size_t>(row)]);
    ImGui::EndTable();
}

}
