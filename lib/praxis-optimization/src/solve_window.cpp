#include "scenario_answers.h"

#include "praxis/optimization/solve_window.h"

#include "praxis/extension/refusal.h"

#include <imgui.h>

#include <span>
#include <array>
#include <format>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace praxis::optimization {

namespace {

constexpr std::array<const char *, 3> residual_columns{"k", "x_k", "e_k"};
constexpr std::array<const char *, 4> objective_columns{"k", "x_k", "f_k", "grad f_k"};

std::string coordinates(const Eigen::VectorXd &x)
{
    std::string written;
    for(Eigen::Index at = 0; at < x.size(); ++at)
        written += std::format("{}{:.12g}", at == 0 ? "" : ", ", x[at]);

    return written;
}

std::string entries(const expected<Eigen::VectorXd, refusal> &e)
{
    if(!e)
        return std::string(refusal_text(e.error()));

    std::string written;
    for(Eigen::Index at = 0; at < e->size(); ++at)
        written += std::format("{}{:.3e}", at == 0 ? "" : ", ", (*e)[at]);

    return written;
}

std::string value(const expected<double, refusal> &f)
{
    return f ? std::format("{:.12g}", *f) : std::string(refusal_text(f.error()));
}

std::vector<std::string> cells_at(const scenario_run &run, std::size_t k)
{
    if(!run.minimized_by)
        return {coordinates(run.iterates[k]), k < run.errors.size() ? entries(run.errors[k]) : std::string()};
    if(k >= run.objectives.size() || k >= run.gradients.size())
        return {coordinates(run.iterates[k]), std::string(), std::string()};

    return {coordinates(run.iterates[k]), value(run.objectives[k]), entries(run.gradients[k])};
}

void render_row(const iterate_row &shown)
{
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::Text("%zu", shown.k);
    for(const std::string &cell : shown.cells)
    {
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(cell.c_str());
    }
}

}

std::vector<iterate_row> iterate_rows(const scenario_run &run)
{
    std::vector<iterate_row> rows;
    for(std::size_t k = 0; k < run.iterates.size(); ++k)
        rows.push_back(iterate_row{k, cells_at(run, k)});

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
        , m_typed_step(inputs.opened.objective.step_length)
        , m_form(inputs.opened.objective.form)
        , m_step_length(inputs.opened.objective.step_length)
        , m_step(inputs.opened.step)
        , m_limit(inputs.opened.limit)
        , m_seed(inputs.opened.seed)
        , m_seen(inputs.seen)
        , m_method(inputs.opened.objective.method)
        , m_rows()
        , m_scenario(inputs.scenario)
        , m_last()
{
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
    const std::span<const char *const> columns = m_last->run.minimized_by ? std::span<const char *const>(objective_columns) : std::span<const char *const>(residual_columns);
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
