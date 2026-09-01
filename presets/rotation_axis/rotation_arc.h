#ifndef HPP_GUARD_PRAXIS_PRESETS_ROTATION_AXIS_ROTATION_ARC_H
#define HPP_GUARD_PRAXIS_PRESETS_ROTATION_AXIS_ROTATION_ARC_H

#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/slots.h"

#include <threepp/core/Object3D.hpp>

#include <threepp/math/Color.hpp>

#include <Eigen/Core>

#include <memory>

namespace praxis::presets {

// The arc the point `carried` has already travelled, from no turn at all to the turn of
// `angle_radians` about `unit_axis` that is now commanded. `carried` is that point in the moving
// object's own coordinates and `start` the pose that object is turned from; the two are carried in
// the order the driving controls apply them, and the points answered are in the z-up frame the
// stencil's poses are written in. The arc is sampled from the same operation and the same start pose
// the object is driven by, so the arc and the object cannot disagree about the motion. A turn of
// nothing is drawn as nothing.
std::shared_ptr<threepp::Object3D> traversed_arc(const rigid_motion::screw_ops &screw, const Eigen::Vector3d &unit_axis, double angle_radians, const transform &start,
                                                 const Eigen::Vector3d &carried, const threepp::Color &tone);

}

#endif
