#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_TYPES_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_TYPES_H

#include "praxis/trajectory/types.h"

#include <Eigen/Core>

namespace praxis::manipulator {

using jacobian     = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using joint_vector = trajectory::configuration;
using joint_limits = trajectory::configuration_limits;

}

#endif
