#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_TYPED_TYPED_PROBLEM_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_TYPED_TYPED_PROBLEM_H

#include "evaluate.h"
#include "expression.h"

#include "praxis/optimization/problem.h"

#include <memory>

namespace praxis::optimization::typed {

// Lynch & Park, Modern Robotics, sec. 6.2: the residual of f(x) = 0 is e = -f and J is grad f as one
// row, converged where |e|_inf < tolerance. A value or a gradient that is not finite is refused as
// degenerate, an x that is not one coordinate per variable as unsupported input.
problem typed_residual(std::shared_ptr<const expression> tree, parameter_values values, double tolerance);

// Minimizes f, stationary where |grad f|_inf < tolerance; the value, the gradient and the Hessian are
// each refused as the residual's are.
problem typed_objective(std::shared_ptr<const expression> tree, parameter_values values, double tolerance);

}

#endif
