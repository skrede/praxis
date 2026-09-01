#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_TYPES_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_TYPES_H

#include <Eigen/Core>

namespace praxis::trajectory {

using configuration = Eigen::VectorXd;

// Declaration order is frozen: the aggregate is written with designated initializers wherever it is
// composed. Every member carries one entry per degree of freedom, in the order the configuration
// vector carries them; the position pair bounds the configuration itself and the other two bound its
// first and second time derivatives.
struct configuration_limits
{
    configuration velocity;
    configuration acceleration;
    configuration lower_position;
    configuration upper_position;
};

}

#endif
