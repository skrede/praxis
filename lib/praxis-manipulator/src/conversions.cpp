#include "praxis/manipulator/conversions.h"

#include <Eigen/Core>

#include <cstddef>

namespace praxis::manipulator {

std::vector<double> to_std_vector(const joint_vector &vector)
{
    return std::vector<double>(vector.data(), vector.data() + vector.size());
}

joint_vector to_joint_vector(const std::vector<double> &vector)
{
    return Eigen::Map<const joint_vector>(vector.data(), static_cast<Eigen::Index>(vector.size()));
}

}
