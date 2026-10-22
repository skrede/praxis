#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_REGISTERED_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_REGISTERED_H

#include "praxis/optimization/problem.h"

#include <Eigen/Core>

#include <string>
#include <optional>
#include <functional>

namespace praxis::optimization {

// A function of one variable a composition hands a scenario under its name, with its derivative
// written out beside it and, where it is given, its second derivative.
struct registered_curve
{
    registered_curve(std::string called, std::function<double(double)> g, std::function<double(double)> slope);
    registered_curve(std::string called, std::function<double(double)> g, std::function<double(double)> slope, box region);
    registered_curve(std::string called, std::function<double(double)> g, std::function<double(double)> slope, std::function<double(double)> second);
    registered_curve(std::string called, std::function<double(double)> g, std::function<double(double)> slope, std::function<double(double)> second, box region);

    std::string name;
    std::optional<box> bounds;
    std::function<double(double)> value;
    std::function<double(double)> derivative;
    std::function<double(double)> second_derivative;
};

// Lynch & Park, Modern Robotics, sec. 6.2.1: the residual of g(x) = 0 is e = -g(x) and J = [g'(x)].
// A value that is not finite is refused as degenerate, an x that is not one coordinate and a curve
// missing a function as unsupported input.
problem curve_problem(const registered_curve &curve, double tolerance);

// Nocedal & Wright, Numerical Optimization, 1st ed., sec. 2.1: minimize f, stationary where
// |grad f|_inf < tolerance. The Hessian is carried only where the second derivative was registered,
// and an answer is refused as the residual's is.
problem curve_objective(const registered_curve &curve, double tolerance);

// A function of two variables a composition hands a scenario under its name, with its gradient
// written out beside it and, where it is given, its Hessian.
struct registered_surface
{
    registered_surface(std::string called, std::function<double(double, double)> f, std::function<Eigen::Vector2d(double, double)> slope);
    registered_surface(std::string called, std::function<double(double, double)> f, std::function<Eigen::Vector2d(double, double)> slope, box region);
    registered_surface(std::string called, std::function<double(double, double)> f, std::function<Eigen::Vector2d(double, double)> slope,
                       std::function<Eigen::Matrix2d(double, double)> curvature);
    registered_surface(std::string called, std::function<double(double, double)> f, std::function<Eigen::Vector2d(double, double)> slope,
                       std::function<Eigen::Matrix2d(double, double)> curvature, box region);

    std::string name;
    std::optional<box> bounds;
    std::function<double(double, double)> value;
    std::function<Eigen::Vector2d(double, double)> gradient;
    std::function<Eigen::Matrix2d(double, double)> hessian;
};

// Lynch & Park, Modern Robotics, sec. 6.2: the residual of f(x, y) = 0 is e = -f and J is the
// gradient as one row. A value or a gradient that is not finite is refused as degenerate, an x that
// is not two coordinates and a surface missing a function as unsupported input.
problem surface_problem(const registered_surface &surface, double tolerance);

// Nocedal & Wright, Numerical Optimization, 1st ed., sec. 2.1: minimize f, stationary where
// |grad f|_inf < tolerance. The Hessian is carried only where it was registered, and an answer is
// refused as the residual's is.
problem surface_objective(const registered_surface &surface, double tolerance);

}

#endif
