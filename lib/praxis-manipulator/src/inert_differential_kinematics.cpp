#include "praxis/manipulator/kinematics.h"

#include <span>

namespace praxis::manipulator::inert {

expected<jacobian, refusal> space_jacobian(std::span<const screw_axis>, const joint_vector &)
{
    return unexpected(refusal::not_implemented);
}

expected<jacobian, refusal> body_jacobian(std::span<const screw_axis>, const joint_vector &)
{
    return unexpected(refusal::not_implemented);
}

}
