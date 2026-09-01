#include "praxis/manipulator/screw_chain.h"

#include <utility>

namespace praxis::manipulator {

// Eigen leaves a fixed-size matrix uninitialized on value initialization, so the identity has to be
// written in rather than defaulted to.
screw_chain::screw_chain()
        : home(transform::Identity())
{
}

screw_chain::screw_chain(transform home_pose, std::vector<screw_axis> screws, joint_limits bounds)
        : home(std::move(home_pose))
        , space_screws(std::move(screws))
        , limits(std::move(bounds))
{
}

std::size_t screw_chain::joint_count() const
{
    return space_screws.size();
}

}
