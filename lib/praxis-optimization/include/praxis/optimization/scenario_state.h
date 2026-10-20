#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_STATE_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_STATE_H

#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scheduler/ownership.h"

#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <utility>

namespace praxis::optimization {

struct scenario_settings
{
    std::string selected;
    interval range;
    double seed;
    std::uint32_t limit;
    double tolerance;
};

// The registered functions and the bound solver are called here and only here, on the strand that
// owns this state, so a function or a solve that is slow or never returns stalls no frame.
class scenario_state
{
public:
    scenario_state(std::vector<registered_curve> curves, capabilities solving, scenario_settings opened, std::shared_ptr<scenario_publisher> published);

    // A selection naming no registered function leaves the last publication standing.
    template<typename operation>
    void mutate(operation work)
    {
        work(m_settings);
        run();
    }

    void run();

private:
    capabilities m_solving;
    scenario_settings m_settings;
    std::vector<registered_curve> m_curves;
    std::shared_ptr<scenario_publisher> m_published;
};

using owned_scenario = scheduler::strand_owned<scenario_state>;

// An expired observer means the preset is gone, so the command is not sent and nothing is reported.
template<typename operation>
void command(const std::weak_ptr<owned_scenario> &scenario, operation work)
{
    const std::shared_ptr<owned_scenario> gated = scenario.lock();
    if(gated == nullptr)
        return;

    static_cast<void>(gated->with([work = std::move(work)](scenario_state &state) mutable { state.mutate(std::move(work)); }));
}

}

#endif
