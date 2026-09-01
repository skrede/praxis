#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_DETAIL_FINITE_DIFFERENCE_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_DETAIL_FINITE_DIFFERENCE_H

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <algorithm>
#include <type_traits>

namespace praxis::trajectory::detail {

constexpr double parameter_step = 1.0e-4;

template<typename T>
struct derivatives
{
    T first_derivative;
    T second_derivative;
};

template<typename Sampled>
struct differenced;

template<typename T>
struct differenced<expected<T, refusal>>
{
    using type = derivatives<T>;
};

template<typename Sample>
using differenced_t = typename differenced<std::invoke_result_t<const Sample &, double>>::type;

// The stencil is pulled inside the unit interval at the ends, where the path is not defined beyond
// it. Both scaling derivatives vanish there, so the shift is not visible. A sampler that refuses at
// any of the three points refuses the whole difference: no derivative is formed over a substitute.
template<typename Sample>
expected<differenced_t<Sample>, refusal> central_differences(const Sample &sampled, double s)
{
    const double centre = std::clamp(s, parameter_step, 1.0 - parameter_step);
    const auto ahead    = sampled(centre + parameter_step);
    const auto here     = sampled(centre);
    const auto behind   = sampled(centre - parameter_step);

    if(!ahead)
        return unexpected(ahead.error());
    if(!here)
        return unexpected(here.error());
    if(!behind)
        return unexpected(behind.error());

    return differenced_t<Sample>{(*ahead - *behind) / (2.0 * parameter_step), (*ahead - 2.0 * (*here) + *behind) / (parameter_step * parameter_step)};
}

}

#endif
