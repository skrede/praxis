#include "praxis/trajectory/trajectory.h"

#include <memory>
#include <vector>

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

expected<std::unique_ptr<trajectory_generator>, refusal> joint_space_via_points(std::span<const via_point>)
{
    return std::make_unique<inert_trajectory_generator>();
}

expected<std::vector<via_point>, refusal> via_point_schedule(std::span<const configuration>, const configuration &, const configuration_limits &)
{
    return unexpected(refusal::not_implemented);
}

}
