#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_REGISTERED_VALUES_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_REGISTERED_VALUES_H

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <utility>
#include <functional>
#include <type_traits>

namespace praxis::optimization {

// A registered function called at x, one coordinate per argument. A function missing, or an x that is
// not one coordinate per argument, is refused as unsupported input, and an answer that is not finite
// as degenerate.
template<typename answer, typename... coordinates>
expected<answer, refusal> evaluated(const std::function<answer(coordinates...)> &f, const Eigen::VectorXd &x)
{
    if(!f || x.size() != static_cast<Eigen::Index>(sizeof...(coordinates)))
        return unexpected(refusal::unsupported_input);

    const answer answered = [&]<std::size_t... at>(std::index_sequence<at...>) { return f(x[static_cast<Eigen::Index>(at)]...); }(std::index_sequence_for<coordinates...>());
    if constexpr(std::is_same_v<answer, double>)
    {
        if(!std::isfinite(answered))
            return unexpected(refusal::degenerate);
    }
    else if(!answered.allFinite())
        return unexpected(refusal::degenerate);

    return answered;
}

}

#endif
