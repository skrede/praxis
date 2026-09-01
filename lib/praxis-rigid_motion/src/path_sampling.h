#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_PATH_SAMPLING_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_PATH_SAMPLING_H

#include "praxis/rigid_motion/types.h"

#include <vector>
#include <cstddef>

namespace praxis::rigid_motion {

// The parameter the sample at that index stands at, zero at the first and one at the last. Every
// path drawn between one pair of poses is sampled through this, so two of them are comparable
// sample for sample rather than only end to end.
inline double sampled_share(std::size_t at, std::size_t points)
{
    return static_cast<double>(at) / static_cast<double>(points - 1);
}

// The length in metres of the polyline through the translation columns of the sampled poses.
inline double path_length(const std::vector<transform> &path)
{
    double along = 0.0;
    for(std::size_t at = 0; at + 1 < path.size(); ++at)
        along += (path[at + 1].block<3, 1>(0, 3) - path[at].block<3, 1>(0, 3)).norm();

    return along;
}

}

#endif
