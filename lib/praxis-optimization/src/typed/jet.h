#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_TYPED_JET_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_TYPED_JET_H

#include "expression.h"

#include <Eigen/Core>

#include <vector>

namespace praxis::optimization::typed {

// A value with its gradient and symmetric Hessian over at most two variables, propagated by second-order
// forward differentiation (Griewank & Walther, Evaluating Derivatives, 2nd ed., ch. 13). In one
// dimension only the first components are read.
struct jet
{
    double value;
    Eigen::Vector2d gradient;
    Eigen::Matrix2d hessian;
};

jet lifted(double value);

// The variable `direction` of the jet's two, at `value`.
jet seeded(double value, Eigen::Index direction);

jet operator+(const jet &a, const jet &b);
jet operator-(const jet &a, const jet &b);
jet operator*(const jet &a, const jet &b);
jet operator/(const jet &a, const jet &b);
jet operator-(const jet &a);

// Whether u carries an exactly zero gradient and Hessian.
bool constant(const jet &u);

// phi(u) from phi's value and its first and second derivatives at u's value; a constant u lifts phi
// alone, so a derivative that is infinite there is never multiplied by u's zero ones.
jet chained(const jet &u, double phi, double d1, double d2);

jet applied(named_function function, const jet &u);

// An exponent with an exactly zero gradient and Hessian is a constant c, whose rule skips a term whose
// coefficient is zero; any other is exp(w log u).
jet raised(const jet &base, const jet &exponent);

// The node `power` over `slots`, the jets of the nodes before it. A constant power c of nested square
// roots of r takes its derivatives from r^(c / 2^roots) where r is not negative, since the two agree
// there and the power can be smooth where a root is not, as sqrt(x^2 + y^2)^2 is at the origin.
jet raised_root(const std::vector<node> &nodes, const std::vector<jet> &slots, const node &power);

}

#endif
