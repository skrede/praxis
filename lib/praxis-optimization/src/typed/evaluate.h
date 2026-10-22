#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_TYPED_EVALUATE_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_TYPED_EVALUATE_H

#include "jet.h"
#include "expression.h"

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <vector>
#include <cstddef>
#include <type_traits>

namespace praxis::optimization::typed {

// A parameter's value, indexed by its letter minus 'a'.
using parameter_values = std::array<double, 26>;

inline double applied(named_function function, double u)
{
    switch(function)
    {
        case named_function::sin:
            return std::sin(u);
        case named_function::cos:
            return std::cos(u);
        case named_function::tan:
            return std::tan(u);
        case named_function::exp:
            return std::exp(u);
        case named_function::log:
            return std::log(u);
        case named_function::sqrt:
            return std::sqrt(u);
    }
    return u;
}

inline double raised(double base, double exponent)
{
    return std::pow(base, exponent);
}

template<typename scalar>
scalar leaf(const node &one, const Eigen::VectorXd &x, const parameter_values &values)
{
    const bool variable          = one.kind == node_kind::variable;
    const Eigen::Index direction = one.letter == 'x' ? 0 : 1;
    const double value           = variable ? x[direction] : one.kind == node_kind::parameter ? values[static_cast<std::size_t>(one.letter - 'a')] : one.value;
    if constexpr(std::is_same_v<scalar, jet>)
        return variable ? seeded(value, direction) : lifted(value);
    else
        return value;
}

template<typename scalar>
scalar combined(const node &one, const std::vector<scalar> &slots, const Eigen::VectorXd &x, const parameter_values &values)
{
    switch(one.kind)
    {
        case node_kind::negate:
            return -slots[one.left];
        case node_kind::function:
            return applied(one.function, slots[one.left]);
        case node_kind::add:
            return slots[one.left] + slots[one.right];
        case node_kind::subtract:
            return slots[one.left] - slots[one.right];
        case node_kind::multiply:
            return slots[one.left] * slots[one.right];
        case node_kind::divide:
            return slots[one.left] / slots[one.right];
        case node_kind::power:
            return raised(slots[one.left], slots[one.right]);
        default:
            return leaf<scalar>(one, x, values);
    }
}

// x holds one coordinate per variable of the tree's dimension; `double` answers the value and `jet`
// the value with its derivatives.
template<typename scalar>
scalar evaluated(const expression &tree, const Eigen::VectorXd &x, const parameter_values &values)
{
    std::vector<scalar> slots;
    slots.reserve(tree.nodes.size());
    for(const node &one : tree.nodes)
        slots.push_back(combined(one, slots, x, values));
    return slots.back();
}

}

#endif
