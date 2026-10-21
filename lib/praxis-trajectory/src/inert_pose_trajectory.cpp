#include "praxis/trajectory/pose_trajectory.h"

#include <memory>

namespace praxis::trajectory::inert {

expected<std::unique_ptr<pose_trajectory_generator>, refusal> decoupled_pose_waypoints(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &, const path_ops &,
                                                                                       const time_scaling_ops &, std::span<const transform>, const transform &, double, double)
{
    return unexpected(refusal::not_implemented);
}

expected<std::unique_ptr<pose_trajectory_generator>, refusal> screw_pose_waypoints(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &, const path_ops &,
                                                                                   const time_scaling_ops &, std::span<const transform>, const transform &, double, double)
{
    return unexpected(refusal::not_implemented);
}

}
