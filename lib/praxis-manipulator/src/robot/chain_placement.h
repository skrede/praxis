#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_CHAIN_PLACEMENT_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_CHAIN_PLACEMENT_H

#include "praxis/manipulator/types.h"

#include "praxis/rigid_motion/screw.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include <Eigen/Core>

#include <span>
#include <vector>
#include <cstddef>

namespace praxis::manipulator {

// The stick figure the told screws imply, in the model's root-link frame: the frame's origin, one
// point per joint, and the point the home transform names once the configuration has been walked.
// Joint i's point is the point of joint i's axis nearest the point before it. A screw with no
// angular part names no axis to project onto and carries the point before it forward, and a pair of
// axes that meet gives one point twice rather than a point fewer, so the count is the joint count
// and two whatever configuration the fold is asked for. It is refused where the home is not finite,
// where fold_chain_end refuses or where the adjoint map refuses; each screw is mapped through the rigid
// motion nearest the product before it, and the last point is where fold_chain_end ends.
expected<std::vector<Eigen::Vector3d>, refusal> fold_joint_origins(const transform &home, std::span<const screw_axis> space_screws, const joint_vector &theta,
                                                                   const rigid_motion::screw_ops &screw);

// The product e^[S1]theta1 ... e^[Sn]thetan M in the model's root-link frame, Lynch & Park, Modern Robotics,
// eq. (4.14), answered only where each exponential's rigidity defect is within the bound; otherwise the joint,
// counted from zero, whose exponential is not. The home transform is not held to the bound.
expected<transform, std::size_t> fold_chain_end(const transform &home, std::span<const screw_axis> space_screws, const joint_vector &theta, const rigid_motion::screw_ops &screw);

// How far a transform stands from a rigid motion: the greater of max |R^T R - I|, |det R - 1| and the
// bottom row's departure from (0, 0, 0, 1). Not a number where an entry is not finite.
double rigidity_defect(const transform &placed);

// The rotation nearest a block in the Frobenius sense, with the last singular direction flipped
// where the decomposition would otherwise answer a reflection.
Eigen::Matrix3d nearest_rotation(const Eigen::Matrix3d &block);

// The rotation nearest the transform's block, its translation, and an exact bottom row.
transform nearest_rigid_motion(const transform &placed);

}

#endif
