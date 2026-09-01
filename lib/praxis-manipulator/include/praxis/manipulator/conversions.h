#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_CONVERSIONS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_CONVERSIONS_H

#include "praxis/manipulator/types.h"

#include <vector>

namespace praxis::manipulator {

std::vector<double> to_std_vector(const joint_vector &vector);

joint_vector to_joint_vector(const std::vector<double> &vector);

}

#endif
