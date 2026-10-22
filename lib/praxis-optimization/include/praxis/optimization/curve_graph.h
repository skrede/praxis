#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_CURVE_GRAPH_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_CURVE_GRAPH_H

#include "praxis/optimization/scenario_view.h"
#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/plot_window.h"

#include <memory>
#include <string>

namespace praxis::optimization {

// The published curve as a line held to its range and every iterate x_k as a marker at (x_k, g(x_k)).
// At the selected step k it adds the tangent at x_k, the point where that tangent is zero, and the
// recorded x_(k+1) as a segment from the abscissa to the curve; where the problem has a box, its ends;
// at a point p asked for, the point and its tangent, and the central difference's tangent where the
// check disagrees. Each added line is held to the range and to the ordinates the points span. The view
// says which of these is drawn beside the curve.
scene::plot_reading curve_reading(const scenario_reader &seen, const scenario_view &drawn);

// A left press with Shift held over the plot runs from the abscissa pressed. The plot reads the view the
// inputs carry.
std::shared_ptr<scene::plot_window> curve_graph(const scenario_window_inputs &inputs, std::string name);

}

#endif
