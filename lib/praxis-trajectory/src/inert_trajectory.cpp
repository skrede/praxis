#include "praxis/trajectory/trajectory.h"

#include <memory>
#include <vector>

namespace praxis::trajectory::inert {

expected<std::unique_ptr<trajectory_generator>, refusal> joint_space_via_points(std::span<const via_point>)
{
    return unexpected(refusal::not_implemented);
}

expected<std::vector<via_point>, refusal> via_point_schedule(std::span<const configuration>, const configuration &, const configuration_limits &)
{
    return unexpected(refusal::not_implemented);
}

}
