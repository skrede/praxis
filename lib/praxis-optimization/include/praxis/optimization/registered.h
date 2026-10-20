#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_REGISTERED_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_REGISTERED_H

#include "praxis/optimization/problem.h"

#include <string>
#include <optional>
#include <functional>

namespace praxis::optimization {

// A function of one variable a composition hands a scenario under its name, with its derivative
// written out beside it.
struct registered_curve
{
    registered_curve(std::string called, std::function<double(double)> g, std::function<double(double)> slope);
    registered_curve(std::string called, std::function<double(double)> g, std::function<double(double)> slope, box region);

    std::string name;
    std::optional<box> bounds;
    std::function<double(double)> value;
    std::function<double(double)> derivative;
};

// Lynch & Park, Modern Robotics, sec. 6.2.1: the residual of g(x) = 0 is e = -g(x) and J = [g'(x)].
// A value that is not finite is refused as degenerate, an x that is not one coordinate and a curve
// missing a function as unsupported input.
problem curve_problem(const registered_curve &curve, double tolerance);

}

#endif
