#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_LINEARIZATION_WINDOW_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_LINEARIZATION_WINDOW_H

#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/imgui_window.h"

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>

namespace praxis::optimization {

// The value and the derivative at the point as the window writes them; where the check is on and was
// evaluated, the central difference, and a line stating a disagreement.
std::vector<std::string> linearization_lines(const scenario_linearization &at, bool checks_derivative);

// A point taken by a field and by a slider over the range, asked of the scenario through its gate as
// either is edited; the window asks for the middle of the range it opened at when it is built.
class linearization_window : public scene::imgui_window
{
public:
    linearization_window(std::string name, const scenario_window_inputs &inputs);

    void render() override;

private:
    bool m_checks;
    Eigen::VectorXd m_point;
    scenario_reader m_seen;
    std::weak_ptr<owned_scenario> m_scenario;

    void post() const;
};

}

#endif
