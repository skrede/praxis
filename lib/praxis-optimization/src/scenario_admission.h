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
#include <string_view>

namespace praxis::optimization {

// One finite range per variable, each with its minimum below its maximum.
bool drawable(std::span<const interval> ranges, std::size_t dimension);

// What a request of a function of `dimension` variables may name, a registered name only where the
// source is registered. Each logs what it refuses, and that the last run stays shown.
std::optional<std::size_t> selected_among(const std::vector<std::string> &names, const std::string &name);
bool drawn_over(std::span<const interval> ranges, std::size_t dimension);
bool taken_at(const Eigen::VectorXd &x, std::size_t dimension, const char *as);
bool admitted(const scenario_settings &edited, const std::vector<std::string> &names, std::size_t dimension);
bool descends_by(double step_length);

// One parameter per letter of `letters` in their order, each as `held` carries it, or at the default
// value and range where `held` carries none.
std::vector<typed_parameter> parameters_for(std::string_view letters, const std::vector<typed_parameter> &held);

// The settings with one parameter per letter their text uses, where it reads as a function of
// `dimension` variables, and as they are where it does not.
typed_settings lettered(typed_settings typed, std::size_t dimension);

// The parameters a run is posed with: the typed function's where the source is manual, none otherwise.
std::vector<typed_parameter> posed_parameters(const typed_settings &typed);

}

#endif
