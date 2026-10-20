#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_SOLVER_ADAPTERS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_SOLVER_ADAPTERS_H

#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/screw_chain.h"

// Each declaration below matches the slot of inverse_kinematics_ops by signature and hands the solver
// it receives the residual problem of Lynch & Park, Modern Robotics, sec. 6.2.2, read through the
// operations it receives: the body form's error is V_b = log(T_sb^-1 T_sd) with J_b, the space form's
// V_s = [Ad_T_sb] V_b with J_s, angular half first. The error has converged when its angular half is
// within orientation_tol and its linear half within position_tol, and the problem is bounded by the
// chain's joint positions, a joint the limits leave uncovered being free. Every iterate the solver
// recorded, x_0 first, is mapped into the answer with both halves of the error at it, NaN where the
// error cannot be read there. A seed of the wrong width is refused unsupported_input, a chain not
// admitted or a target that is not a rigid motion degenerate, and a seed not finite no_solution; a
// refusal of the solver's other than not_implemented or unsupported_input is answered no_solution.
namespace praxis::manipulator {

expected<void, refusal> body_inverse_kinematics(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const forward_kinematics_ops &forward,
                                                const differential_kinematics_ops &differential, const optimization::solver_ops &solver, const screw_chain &chain,
                                                const transform &desired, const joint_vector &j0, const solver_parameters &parameters, ik_result &answer);

expected<void, refusal> space_inverse_kinematics(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const forward_kinematics_ops &forward,
                                                 const differential_kinematics_ops &differential, const optimization::solver_ops &solver, const screw_chain &chain,
                                                 const transform &desired, const joint_vector &j0, const solver_parameters &parameters, ik_result &answer);

}

#endif
