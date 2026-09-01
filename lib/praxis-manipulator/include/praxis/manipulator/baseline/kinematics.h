#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_BASELINE_KINEMATICS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_BASELINE_KINEMATICS_H

#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/screw_chain.h"

#include <span>
#include <vector>

// Every declaration below matches a slot of one of the kinematic aggregates by name and by signature,
// so composing an aggregate is a plain address-of.
namespace praxis::manipulator {

expected<transform, refusal> forward_kinematics(const transform &m, std::span<const screw_axis> space_screws, const joint_vector &theta);

expected<jacobian, refusal> space_jacobian(std::span<const screw_axis> space_screws, const joint_vector &theta);
expected<jacobian, refusal> body_jacobian(std::span<const screw_axis> body_screws, const joint_vector &theta);

expected<std::vector<screw_axis>, refusal> body_screws_from_space(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const transform &m,
                                                                  std::span<const screw_axis> space_screws);

expected<void, refusal> inverse_kinematics(const forward_kinematics_ops &forward, const differential_kinematics_ops &differential, const screw_chain &chain, const transform &desired,
                                           const joint_vector &j0, const solver_parameters &parameters, ik_result &answer);
expected<void, refusal> analytic_inverse_kinematics(const forward_kinematics_ops &forward, const screw_chain &chain, const transform &desired, ik_result &answer);

expected<transform, refusal> body_forward_kinematics(const rigid_motion::frame_ops &frames, const transform &m, std::span<const screw_axis> body_screws, const joint_vector &theta);

// A chain held against the bindings the caller hands over, so the choice of them is made where this
// is called. A chain the solver library cannot represent -- no joints, a home pose that is not a
// rigid motion, or a screw axis that is not unit-normalized -- is refused rather than held.
expected<kinematics, refusal> make_kinematics(const screw_chain &chain, forward_kinematics_ops forward, differential_kinematics_ops differential, inverse_kinematics_ops inverse,
                                              const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames);

}

#endif
