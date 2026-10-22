#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SOLVE_WINDOW_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SOLVE_WINDOW_H

#include "praxis/optimization/slots.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include <Eigen/Core>

#include <span>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace praxis::optimization {

// One row of the iterate table: k, then x_k with e_k, or where the run was minimized x_k with f_k and
// grad f_k, as it writes them; an answer the problem refused is written as the refusal.
struct iterate_row
{
    std::size_t k;
    std::vector<std::string> cells;
};

std::vector<iterate_row> iterate_rows(const scenario_run &run);

// The steps the run took and what it came to: converged, answered where the problem has not
// converged, stepped short of the limit, or refused.
std::string run_status(const scenario_snapshot &shown);

// The names the minimizers are offered under, in minimizer_slot order.
std::span<const char *const> minimizer_labels();

// The seed, the iteration limit and the selected step are held here and asked of the scenario
// through its gate: the seed goes with Run, Step once and Reset, the limit and the step as they are
// edited. In the minimum form the method and the step length are held and asked for as well, a step
// length once its field is left after an edit; the last step length the scenario can take is the one
// held. Their keys sit at the root of the scenario's document, and the form's is never written here.
class solve_window : public scene::imgui_window, public config::configurable
{
public:
    solve_window(std::string name, const scenario_window_inputs &inputs);

    void render() override;

    std::string_view settings_path() const override
    {
        return {};
    }

    std::vector<config::edit> settings_edits(const config::document &carried) const override;

    const config::configurable *as_configurable() const override
    {
        return this;
    }

private:
    double m_typed_step;
    problem_form m_form;
    double m_step_length;
    std::uint32_t m_step;
    std::uint32_t m_limit;
    Eigen::VectorXd m_seed;
    scenario_reader m_seen;
    minimizer_slot m_method;
    std::vector<iterate_row> m_rows;
    std::weak_ptr<owned_scenario> m_scenario;
    std::shared_ptr<const scenario_snapshot> m_last;

    void render_controls();
    void render_objective_controls();
    void render_run(const std::shared_ptr<const scenario_snapshot> &shown);
    void render_slider(const scenario_run &run);
    void render_table() const;
};

}

#endif
