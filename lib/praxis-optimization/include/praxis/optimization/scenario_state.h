#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_STATE_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_STATE_H

#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scheduler/ownership.h"

#include <Eigen/Core>

#include <span>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <concepts>
#include <optional>

namespace praxis::optimization {

// One range and one seed coordinate per variable of the function; `step` is the step k selected,
// from x_k to x_(k+1).
struct scenario_settings
{
    std::string selected;
    std::vector<interval> ranges;
    Eigen::VectorXd seed;
    std::uint32_t limit;
    std::uint32_t step;
};

// The registered functions and the bound solver are called here and only here, on the strand that
// owns this state, so a function or a solve that is slow or never returns stalls no frame. A request
// naming a seed, a point or a range the function cannot take is logged and changes nothing. The
// central difference of a linearization is evaluated only where the state checks derivatives.
class scenario_state
{
public:
    scenario_state(std::vector<registered_curve> curves, capabilities solving, scenario_settings opened, std::shared_ptr<scenario_publisher> published, bool checks_derivative);
    scenario_state(std::vector<registered_surface> surfaces, capabilities solving, scenario_settings opened, std::shared_ptr<scenario_publisher> published, bool checks_derivative);

    // An edit leaving a selection, a range or a seed the requests below would refuse is logged, and
    // the settings stay as they were.
    template<typename operation>
    void mutate(operation work)
    {
        scenario_settings edited = m_settings;
        work(edited);
        if(!admits(edited))
            return;

        m_settings = std::move(edited);
        run();
    }

    void run();
    void run(const Eigen::VectorXd &seed, std::uint32_t limit);

    // Solves again from `seed` with one step more than the shown run took from it, or with one step
    // from a seed it did not start at, never past the iteration limit; the newest step is selected.
    void step_once(const Eigen::VectorXd &seed);

    // Held to the steps the shown run took, and published without solving again.
    void select_step(std::uint32_t step);

    // Taken by the next run; nothing is solved again.
    void set_limit(std::uint32_t limit);

    // Evaluated at once and with every later run, which publishes it beside the solve.
    void linearize(const Eigen::VectorXd &point);

    void select(std::size_t index);
    void set_range(std::span<const interval> ranges);

private:
    bool m_checks;
    std::size_t m_dimension;
    capabilities m_solving;
    scenario_settings m_settings;
    std::vector<std::string> m_names;
    std::vector<problem> m_problems;
    std::optional<Eigen::VectorXd> m_point;
    std::shared_ptr<scenario_publisher> m_published;
    std::shared_ptr<const scenario_snapshot> m_shown;

    scenario_state(std::size_t dimension, std::vector<std::string> names, std::vector<problem> problems, capabilities solving, scenario_settings opened,
                   std::shared_ptr<scenario_publisher> published, bool checks_derivative);

    void solve(std::uint32_t steps, std::uint32_t selecting);
    void republish(std::shared_ptr<scenario_snapshot> moved);
    bool admits(const scenario_settings &edited) const;
};

using owned_scenario = scheduler::strand_owned<scenario_state>;

// An expired observer means the preset is gone, so the command is not sent and nothing is reported.
// An operation taking the state is a request made of it; one taking the settings edits them and runs.
template<typename operation>
void command(const std::weak_ptr<owned_scenario> &scenario, operation work)
{
    const std::shared_ptr<owned_scenario> gated = scenario.lock();
    if(gated == nullptr)
        return;

    static_cast<void>(gated->with(
            [work = std::move(work)](scenario_state &state) mutable
            {
                if constexpr(std::invocable<operation &, scenario_state &>)
                    work(state);
                else
                    state.mutate(std::move(work));
            }));
}

}

#endif
