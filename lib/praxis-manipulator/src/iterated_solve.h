#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ITERATED_SOLVE_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ITERATED_SOLVE_H

#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/screw_chain.h"

#include <span>

namespace praxis::manipulator {

struct body_target
{
    const screw_chain &chain;
    const transform &desired;
    std::span<const screw_axis> body;
};

// V_b = log(T_sb(theta)^-1 T_sd), T_sb = M e^[B_1]theta_1 ... e^[B_n]theta_n: Lynch & Park, Modern
// Robotics, sec. 6.2.2 and eq. (4.16). The angular half leads.
expected<twist, refusal> body_error(const body_target &target, const joint_vector &theta);

// Handed the target, theta, and J_b and V_b at theta, answers the next configuration, one entry per joint, or a refusal.
using solve_step = expected<joint_vector, refusal> (*)(const body_target &target, const joint_vector &theta, const jacobian &jb, const twist &error);

// Refuses, in order, a seed of the wrong width (unsupported_input), a chain or target not admitted (degenerate), a seed not
// finite (no_solution) and body screws not derivable (degenerate); a refused step or one of another width is no_solution.
// Every step at whose configuration V_b can be read is appended to answer.iterations, answered or not; only a converged
// configuration named inside the joint bounds is answered.
expected<void, refusal> iterated(const screw_chain &chain, const transform &desired, const joint_vector &j0, const solver_parameters &parameters, solve_step step, ik_result &answer);

}

#endif
