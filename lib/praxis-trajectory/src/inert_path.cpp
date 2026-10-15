#include "praxis/trajectory/path.h"

namespace praxis::trajectory::inert {

expected<configuration, refusal> joint_straight_line(const configuration &, const configuration &, double)
{
    return unexpected(refusal::not_implemented);
}

expected<transform, refusal> screw(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &, const transform &, const transform &, double)
{
    return unexpected(refusal::not_implemented);
}

expected<transform, refusal> decoupled(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &, const transform &, const transform &, double)
{
    return unexpected(refusal::not_implemented);
}

}
