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

// The points a fold draws the chain through, and the transform its walk reached: the product of
// the exponentials times the home transform.
struct chain_fold
{
    std::vector<Eigen::Vector3d> points;
    transform reached;
};

// The joint a fold was refused at, counted from zero, and the refusal answered there.
struct refused_joint
{
    std::size_t joint;
    refusal reason;
};

// The stick figure the told screws imply, in the model's root-link frame: the frame's origin, one
// point per joint, and the point the home transform names once the configuration has been walked,
// beside the transform that walk reached. Joint i's point is the point of joint i's axis nearest the
// point before it. A screw with no angular part names no axis to project onto and carries the point
// before it forward, and a pair of axes that meet gives one point twice rather than a point fewer, so
// the count is the joint count and two whatever configuration the fold is asked for.
expected<chain_fold, refused_joint> fold_joint_origins(const transform &home, std::span<const screw_axis> space_screws, const joint_vector &theta, const rigid_motion::screw_ops &screw);

}

#endif
