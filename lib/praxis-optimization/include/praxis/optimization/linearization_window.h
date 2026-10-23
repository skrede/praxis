#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_LINEARIZATION_WINDOW_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_LINEARIZATION_WINDOW_H

#include "praxis/optimization/scenario_view.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/imgui_window.h"

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>

namespace praxis::optimization {

// The value and the derivative, or for a function of two variables the gradient, at the point as the
// window writes them; where the check is on and was evaluated, the central difference, and a line
// stating a disagreement. A function of one variable is named f in the minimum form and g in the
// root form.
std::vector<std::string> linearization_lines(const scenario_linearization &at, problem_form form, bool checks_derivative);

// A point taken by a field and by a slider over the range of each variable, asked of the scenario
// through its gate as a slider moves and once a field is left after an edit; the window asks for the
// middle of the ranges it opened at when it is built, and for the middle of the ranges last published
// on Reset. Its Show switch is the view's linearization, which is not saved.
class linearization_window : public scene::imgui_window
{
public:
    linearization_window(std::string name, const scenario_window_inputs &inputs);

    void render() override;

private:
    bool m_checks;
    Eigen::VectorXd m_point;
    scenario_reader m_seen;
    std::shared_ptr<scenario_view> m_drawn;
    std::weak_ptr<owned_scenario> m_scenario;

    void controls(const std::vector<interval> &ranges);
    bool edit_point(const std::vector<interval> &ranges);
    void post() const;
};

}

#endif
