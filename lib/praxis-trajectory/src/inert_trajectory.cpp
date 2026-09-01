#include "praxis/trajectory/trajectory.h"

#include <memory>

namespace praxis::trajectory::inert {

namespace {

class inert_trajectory_generator : public trajectory_generator
{
public:
    expected<trajectory_sample, refusal> sample(double) const override
    {
        return unexpected(refusal::not_implemented);
    }

    double duration() const override
    {
        return 0.0;
    }
};

}

expected<std::unique_ptr<trajectory_generator>, refusal> joint_space_waypoints(std::span<const configuration>, const configuration &, const configuration_limits &)
{
    return std::make_unique<inert_trajectory_generator>();
}

}
