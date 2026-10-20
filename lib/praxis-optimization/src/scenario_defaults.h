#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_DEFAULTS_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SCENARIO_DEFAULTS_H

#include <cstddef>

namespace praxis::optimization {

// The |g(x)| a curve's solve stops below, in the units of g.
inline constexpr double curve_tolerance = 1.0e-12;

// The points a curve is sampled at over its range, ends included, a count.
inline constexpr std::size_t curve_samples = 1601;

}

#endif
