#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SOLVE_WINDOW_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SOLVE_WINDOW_H

#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace praxis::optimization {

// One row of the iterate table, k with x_k and e_k as it writes them; an error the problem refused
// is written as the refusal.
struct iterate_row
{
    std::size_t k;
    std::string x;
    std::string e;
};

std::vector<iterate_row> iterate_rows(const scenario_run &run);

// The steps the run took and what it came to: converged, stepped short of the limit, or refused.
std::string run_status(const scenario_snapshot &shown);

// The seed, the iteration limit and the selected step are held here and asked of the scenario
// through its gate: the seed goes with Run and Step once, the limit and the step as they are edited.
// Their keys sit at the root of the scenario's document.
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
    std::uint32_t m_step;
    std::uint32_t m_limit;
    Eigen::VectorXd m_seed;
    scenario_reader m_seen;
    std::vector<iterate_row> m_rows;
    std::weak_ptr<owned_scenario> m_scenario;
    std::shared_ptr<const scenario_snapshot> m_last;

    void render_controls();
    void render_run(const std::shared_ptr<const scenario_snapshot> &shown);
    void render_slider(const scenario_run &run);
    void render_table() const;
};

}

#endif
