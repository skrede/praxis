#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_ADMISSION_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_ADMISSION_H

#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/scenario_snapshot.h"

#include <Eigen/Core>

#include <span>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>

namespace praxis::optimization {

// One finite range per variable, each with its minimum below its maximum.
bool drawable(std::span<const interval> ranges, std::size_t dimension);

// What a request of a function of `dimension` variables may name. Each logs what it refuses, and
// that the last run stays shown.
std::optional<std::size_t> selected_among(const std::vector<std::string> &names, const std::string &name);
bool drawn_over(std::span<const interval> ranges, std::size_t dimension);
bool taken_at(const Eigen::VectorXd &x, std::size_t dimension, const char *as);
bool admitted(const scenario_settings &edited, const std::vector<std::string> &names, std::size_t dimension);

}

#endif
