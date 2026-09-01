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

expected<scaling_sample, refusal> trapezoidal(double, double, double, double)
{
    return unexpected(refusal::not_implemented);
}

}
