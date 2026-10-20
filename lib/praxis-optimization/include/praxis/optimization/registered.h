#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_REGISTERED_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_REGISTERED_H

#include "praxis/optimization/problem.h"

#include <Eigen/Core>

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

// A function of two variables a composition hands a scenario under its name, with its gradient
// written out beside it.
struct registered_surface
{
    registered_surface(std::string called, std::function<double(double, double)> f, std::function<Eigen::Vector2d(double, double)> slope);
    registered_surface(std::string called, std::function<double(double, double)> f, std::function<Eigen::Vector2d(double, double)> slope, box region);

    std::string name;
    std::optional<box> bounds;
    std::function<double(double, double)> value;
    std::function<Eigen::Vector2d(double, double)> gradient;
};

// Lynch & Park, Modern Robotics, sec. 6.2: the residual of f(x, y) = 0 is e = -f and J is the
// gradient as one row. A value or a gradient that is not finite is refused as degenerate, an x that
// is not two coordinates and a surface missing a function as unsupported input.
problem surface_problem(const registered_surface &surface, double tolerance);

}

#endif
