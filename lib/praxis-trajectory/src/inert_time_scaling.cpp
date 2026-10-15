#include "praxis/trajectory/time_scaling.h"

namespace praxis::trajectory::inert {

expected<scaling_sample, refusal> cubic(double, double)
{
    return unexpected(refusal::not_implemented);
}

expected<scaling_sample, refusal> quintic(double, double)
{
    return unexpected(refusal::not_implemented);
}

expected<scaling_sample, refusal> trapezoidal_from_velocity_acceleration(double, double, double)
{
    return unexpected(refusal::not_implemented);
}

expected<scaling_sample, refusal> trapezoidal_from_velocity_duration(double, double, double)
{
    return unexpected(refusal::not_implemented);
}

expected<scaling_sample, refusal> trapezoidal_from_acceleration_duration(double, double, double)
{
    return unexpected(refusal::not_implemented);
}

expected<scaling_sample, refusal> trapezoidal_within_bounds(double, double, double, double)
{
    return unexpected(refusal::not_implemented);
}

}
