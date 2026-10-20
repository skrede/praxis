#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_CURVE_GRAPH_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_CURVE_GRAPH_H

#include "praxis/optimization/compose_scenario.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scene/plot_window.h"

#include <memory>
#include <string>

namespace praxis::optimization {

// The published curve as a line held to its range and every iterate x_k as a marker at (x_k, g(x_k)).
scene::plot_reading curve_reading(const scenario_reader &seen);

std::shared_ptr<scene::plot_window> curve_graph(const scenario_window_inputs &inputs, std::string name);

}

#endif
