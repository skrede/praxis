#include "jet.h"

#include <cmath>
#include <vector>
#include <cstddef>

namespace praxis::optimization::typed {
namespace {

Eigen::Matrix2d symmetric_product(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
{
    return a * b.transpose() + b * a.transpose();
}

jet reciprocal(const jet &u)
{
    const double r = 1.0 / u.value;
    return chained(u, r, -r * r, 2.0 * r * r * r);
}

jet tangent(const jet &u)
{
    const double t = std::tan(u.value);
    return chained(u, t, 1.0 + t * t, 2.0 * t * (1.0 + t * t));
}

jet root(const jet &u)
{
    const double r = std::sqrt(u.value);
    return chained(u, r, 0.5 / r, -0.25 / (u.value * r));
}

jet constant_power(const jet &u, double c)
{
    const double second = c * (c - 1.0);
    const double d1     = c == 0.0 ? 0.0 : c * std::pow(u.value, c - 1.0);
    const double d2     = second == 0.0 ? 0.0 : second * std::pow(u.value, c - 2.0);
    return chained(u, std::pow(u.value, c), d1, d2);
}

}

jet lifted(double value)
{
    return jet{value, Eigen::Vector2d::Zero(), Eigen::Matrix2d::Zero()};
}

jet seeded(double value, Eigen::Index direction)
{
    jet variable                 = lifted(value);
    variable.gradient[direction] = 1.0;
    return variable;
}

jet operator+(const jet &a, const jet &b)
{
    return jet{a.value + b.value, a.gradient + b.gradient, a.hessian + b.hessian};
}

jet operator-(const jet &a, const jet &b)
{
    return jet{a.value - b.value, a.gradient - b.gradient, a.hessian - b.hessian};
}

jet operator*(const jet &a, const jet &b)
{
    return jet{a.value * b.value, a.gradient * b.value + a.value * b.gradient, a.hessian * b.value + a.value * b.hessian + symmetric_product(a.gradient, b.gradient)};
}

jet operator/(const jet &a, const jet &b)
{
    return a * reciprocal(b);
}

jet operator-(const jet &a)
{
    return jet{-a.value, -a.gradient, -a.hessian};
}

bool constant(const jet &u)
{
    return (u.gradient.array() == 0.0).all() && (u.hessian.array() == 0.0).all();
}

jet chained(const jet &u, double phi, double d1, double d2)
{
    if(constant(u))
        return lifted(phi);

    return jet{phi, d1 * u.gradient, d1 * u.hessian + d2 * u.gradient * u.gradient.transpose()};
}

jet applied(named_function function, const jet &u)
{
    const double v = u.value;
    switch(function)
    {
        case named_function::sin:
            return chained(u, std::sin(v), std::cos(v), -std::sin(v));
        case named_function::cos:
            return chained(u, std::cos(v), -std::sin(v), -std::cos(v));
        case named_function::tan:
            return tangent(u);
        case named_function::exp:
            return chained(u, std::exp(v), std::exp(v), std::exp(v));
        case named_function::log:
            return chained(u, std::log(v), 1.0 / v, -1.0 / (v * v));
        case named_function::sqrt:
            return root(u);
    }
    return lifted(v);
}

jet raised(const jet &base, const jet &exponent)
{
    if(constant(exponent))
        return constant_power(base, exponent.value);

    const double power = std::pow(base.value, exponent.value);
    return chained(exponent * applied(named_function::log, base), power, power, power);
}

jet raised_root(const std::vector<node> &nodes, const std::vector<jet> &slots, const node &power)
{
    const jet plain   = raised(slots[power.left], slots[power.right]);
    std::size_t inner = power.left;
    double halved     = slots[power.right].value;
    for(; nodes[inner].kind == node_kind::function && nodes[inner].function == named_function::sqrt; inner = nodes[inner].left)
        halved *= 0.5;
    if(inner == power.left || !constant(slots[power.right]) || !(slots[inner].value >= 0.0))
        return plain;

    const jet peeled = constant_power(slots[inner], halved);
    return jet{plain.value, peeled.gradient, peeled.hessian};
}

}
