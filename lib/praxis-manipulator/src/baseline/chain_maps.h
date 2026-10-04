#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_BASELINE_CHAIN_MAPS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_BASELINE_CHAIN_MAPS_H

#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/screw_chain.h"

#include <span>
#include <vector>

namespace praxis::manipulator {

bool is_a_rigid_motion(const transform &tf);
bool is_a_rigid_motion(const rigid_motion::frame_ops &frames, const transform &tf);

// No joints, a value that is not finite, a home pose that is not a rigid motion, or a screw axis whose
// angular half (or, where that is zero, linear half) is not of unit length is not admitted.
bool is_admitted(const screw_chain &chain);

// B_i = [Ad_{M^-1}] S_i: Lynch & Park, Modern Robotics, sec. 4.1.3, eq. (4.16).
expected<std::vector<screw_axis>, refusal> to_body_screws(const rigid_motion::screw_ops &screw, const transform &m, std::span<const screw_axis> space_screws);

// e^[S_1]theta_1 ... e^[S_n]theta_n over one screw per joint value.
transform exponential_product(std::span<const screw_axis> screws, const joint_vector &theta);

expected<jacobian, refusal> body_columns(std::span<const screw_axis> body, const joint_vector &theta);

}

#endif
