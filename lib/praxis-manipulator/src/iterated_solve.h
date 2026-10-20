#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ITERATED_SOLVE_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ITERATED_SOLVE_H

#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/screw_chain.h"

namespace praxis::manipulator {

// Lynch & Park, Modern Robotics, sec. 6.2.2, over T_sb, J_b and V_b = log(T_sb^-1 T_sd) read through the
// operations handed in. Refuses, in order, a seed of the wrong width (unsupported_input), a chain not
// admitted or a target not a rigid motion under the handed frames (degenerate) and a seed not finite
// (no_solution). x_0 is recorded first, then every step at whose configuration V_b can be read; the
// configuration at which both halves of V_b are within the tolerances is answered as it is, and a
// refused Jacobian or V_b, or no convergence within the iteration limit, is no_solution.
expected<void, refusal> iterated(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const forward_kinematics_ops &forward,
                                 const differential_kinematics_ops &differential, const screw_chain &chain, const transform &desired, const joint_vector &j0,
                                 const solver_parameters &parameters, ik_result &answer);

}

#endif
