#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_MANIPULABILITY_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_MANIPULABILITY_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/arm_snapshot.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <Eigen/Core>

namespace praxis::manipulator {

// The ellipsoid one 3-by-n block of a Jacobian names. A block that is not three rows, or narrower
// than three columns, names no ellipsoid at all, and neither does one carrying an entry that is not
// a finite number.
expected<manipulability_ellipsoid, refusal> ellipsoid_of(const Eigen::Ref<const Eigen::MatrixXd> &block);

// The angular and the linear ellipsoid of one taken Jacobian, both carrying the Jacobian's own
// refusal where it is a refusal.
jacobian_manipulability manipulability_of(const expected<jacobian, refusal> &taken);

// The flange's body Jacobian re-expressed in the tool frame, the offset being the tool frame's pose
// in the flange frame, and the flange's own refusal where it is one. Lynch & Park, Modern Robotics,
// Def. 3.20.
expected<jacobian, refusal> body_jacobian_at_the_tool(const expected<jacobian, refusal> &flange, const transform &offset);

}

#endif
