#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_ELLIPSOID_PLACEMENT_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_ELLIPSOID_PLACEMENT_H

#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include <threepp/core/Object3D.hpp>

#include <threepp/materials/Material.hpp>

#include <Eigen/Core>

#include <span>
#include <memory>
#include <optional>

namespace praxis::manipulator {

// The tones one ellipsoid is worn from, one entry per step of the condition-number ramp: the body's,
// and the continuation lines'.
struct ellipsoid_tones
{
    std::span<const std::shared_ptr<threepp::Material>> body;
    std::span<const std::shared_ptr<threepp::Material>> line;
};

// Neither the body nor any of the lines is drawn.
void hide_ellipsoid_block(threepp::Object3D &body, std::span<const std::shared_ptr<threepp::Object3D>> lines);

// The drawn length of the semi-axis along principal axis i: the scale times singular value i under
// the velocity reading, and the scale over it under the force reading. Nothing is clamped and no cap
// is taken here, so a reading that runs away answers a length that is not finite and a caller can
// see that it did. This is the one place the rule is written, so a number read and a body drawn
// cannot disagree. Lynch & Park, Modern Robotics, section 5.4.
Eigen::Vector3d drawn_semi_axes(const manipulability_ellipsoid &block, ellipsoid_view read, double scale);

// The body is shaped to the semi-axes the rule above gives. A cap named cuts it flat at the cap and
// continues every axis whose drawn semi-axis strictly exceeds it with a line either way; no cap
// leaves the body whole and every line undrawn. A block that is a refusal, and one whose drawn
// semi-axes are not all finite, is not drawn at all. Line 2*axis runs along principal axis axis and
// line 2*axis+1 runs against it.
void place_ellipsoid_block(const expected<manipulability_ellipsoid, refusal> &block, const Eigen::Vector3d &at, ellipsoid_view read, double scale, const std::optional<double> &cap,
                           threepp::Object3D &body, std::span<const std::shared_ptr<threepp::Object3D>> lines, const ellipsoid_tones &tone);

}

#endif
