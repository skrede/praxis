#include "praxis/trajectory/baseline/time_scaling.h"

#include <ctrlpp/trajectory/cubic_trajectory.h>
#include <ctrlpp/trajectory/quintic_trajectory.h>
#include <ctrlpp/trajectory/trajectory_types.h>
#include <ctrlpp/trajectory/trapezoidal_trajectory.h>

namespace praxis::trajectory {

namespace {

using scalar = ctrlpp::Vector<double, 1>;

expected<scaling_sample, refusal> reported(const ctrlpp::trajectory_point<double, 1> &point)
{
    return scaling_sample{point.position[0], point.velocity[0], point.acceleration[0]};
}

expected<scaling_sample, refusal> refused()
{
    return unexpected(refusal::unsupported_input);
}

}

// Lynch & Park, Modern Robotics, sec. 9.2.2: s(0) = 0, s(T) = 1 and zero velocity at both ends.
expected<scaling_sample, refusal> cubic(double t, double duration)
{
    const auto profile = ctrlpp::make_cubic_trajectory<double, 1>(scalar::Zero(), scalar::Ones(), scalar::Zero(), scalar::Zero(), duration);

    return profile.has_value() ? reported(profile->evaluate(t)) : refused();
}

// Lynch & Park, Modern Robotics, sec. 9.2.2: as the cubic, with zero acceleration at both ends too.
expected<scaling_sample, refusal> quintic(double t, double duration)
{
    const auto profile = ctrlpp::make_quintic_trajectory<double, 1>(scalar::Zero(), scalar::Ones(), scalar::Zero(), scalar::Zero(), scalar::Zero(), scalar::Zero(), duration);

    return profile.has_value() ? reported(profile->evaluate(t)) : refused();
}

// Lynch & Park, Modern Robotics, sec. 9.2.2. The bounds decide the profile's own duration, and the
// requested one is reached by slowing it down; a duration the bounds cannot stretch to is refused.
expected<scaling_sample, refusal> trapezoidal(double t, double duration, double max_velocity, double max_acceleration)
{
    auto profile = ctrlpp::trapezoidal_trajectory<double>::create({.q0 = 0.0, .q1 = 1.0, .v_max = max_velocity, .a_max = max_acceleration});
    if(!profile.has_value() || !profile->rescale_to(duration).has_value())
        return refused();

    return reported(profile->evaluate(t));
}

}
