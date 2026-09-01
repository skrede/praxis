#include "praxis/manipulator/task_trajectory.h"

#include <memory>

namespace praxis::manipulator::inert {

expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> task_space_waypoints(const kinematics &, std::span<const transform>, const joint_vector &, const joint_limits &)
{
    return unexpected(refusal::not_implemented);
}

}
