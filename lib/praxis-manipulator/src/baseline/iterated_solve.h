#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_BASELINE_ITERATED_SOLVE_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_BASELINE_ITERATED_SOLVE_H

#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/screw_chain.h"

#include <span>
#include <vector>

namespace praxis::manipulator {

// The joint step taken from J_b(theta) and V_b(theta), both in the body frame.
using solve_step = joint_vector (*)(const jacobian &jb, const twist &error);

// Refuses, in order, a seed of the wrong width (unsupported_input), a chain or target not admitted (degenerate), a seed not finite (no_solution).
expected<std::vector<screw_axis>, refusal> admitted_body(const screw_chain &chain, const transform &desired, const joint_vector &j0);

// Iterates from j0 by step; only a converged configuration named inside the joint bounds is answered, anything else is no_solution.
expected<void, refusal> solved_inside_bounds(const screw_chain &chain, std::span<const screw_axis> body, const transform &desired, const joint_vector &j0,
                                             const solver_parameters &parameters, solve_step step, ik_result &answer);

}

#endif
