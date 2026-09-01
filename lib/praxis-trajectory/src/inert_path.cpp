#include "praxis/trajectory/path.h"

namespace praxis::trajectory::inert {

expected<configuration, refusal> joint_straight_line(const configuration &, const configuration &, double)
{
    return unexpected(refusal::not_implemented);
}

expected<transform, refusal> screw(const transform &, const transform &, double)
{
    return unexpected(refusal::not_implemented);
}

expected<transform, refusal> decoupled(const transform &, const transform &, double)
{
    return unexpected(refusal::not_implemented);
}

}
