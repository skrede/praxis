#include "praxis/trajectory/pose_trajectory.h"

#include <memory>

namespace praxis::trajectory::inert {

namespace {

// The generator an unbound slot hands back holds no plan, so it reports no duration and answers no
// sample at all: a pose reported here would be indistinguishable from one a bound generator produced.
class inert_pose_trajectory_generator : public pose_trajectory_generator
{
public:
    expected<pose_sample, refusal> sample(double) const override
    {
        return unexpected(refusal::not_implemented);
    }

    double duration() const override
    {
        return 0.0;
    }
};

}

expected<std::unique_ptr<pose_trajectory_generator>, refusal> decoupled_pose_waypoints(std::span<const transform>, const transform &, double, double)
{
    return std::make_unique<inert_pose_trajectory_generator>();
}

expected<std::unique_ptr<pose_trajectory_generator>, refusal> screw_pose_waypoints(std::span<const transform>, const transform &, double, double)
{
    return std::make_unique<inert_pose_trajectory_generator>();
}

}
