#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_BASELINE_BOOK_LOGARITHM_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_BASELINE_BOOK_LOGARITHM_H

#include "praxis/rigid_motion/types.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <Eigen/Core>

#include <utility>

namespace praxis::rigid_motion::book {

expected<std::pair<Eigen::Vector3d, double>, refusal> matrix_logarithm_so3(const rotation &r);
expected<std::pair<screw_axis, double>, refusal> matrix_logarithm_se3_rp(const rotation &r, const Eigen::Vector3d &p);
expected<std::pair<screw_axis, double>, refusal> matrix_logarithm_se3(const transform &tf);

}

#endif
