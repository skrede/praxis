#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_MINIMIZERS_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_MINIMIZERS_H

#include "praxis/optimization/problem.h"
#include "praxis/optimization/minimizer.h"

#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/Cholesky>

#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string_view>

// Nocedal & Wright, Numerical Optimization, 1st ed., sec. 2.2: Newton's method, x_{k+1} = x_k + p_k with
// p_k = -(grad^2 f_k)^-1 grad f_k, and steepest descent, p_k = -alpha grad f_k, each iterated until the
// gradient is stationary or the cap is reached.
namespace praxis::tests::book {

enum class newton_step : std::uint8_t
{
    inverse,
    ldlt,
    partial_lu,
    cod,
    householder_qr
};

enum class newton_cap : std::uint8_t
{
    handed,
    twenty
};

enum class descent_cap : std::uint8_t
{
    handed,
    thousand
};

template<newton_step S>
Eigen::VectorXd newton_step_along(const Eigen::MatrixXd &h, const Eigen::VectorXd &g)
{
    if constexpr(S == newton_step::inverse)
        return h.inverse() * g;
    else if constexpr(S == newton_step::ldlt)
        return h.ldlt().solve(g);
    else if constexpr(S == newton_step::partial_lu)
        return h.partialPivLu().solve(g);
    else if constexpr(S == newton_step::cod)
        return Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>(h).solve(g);
    else
        return h.colPivHouseholderQr().solve(g);
}

template<newton_step S, newton_cap C>
expected<Eigen::VectorXd, refusal> newton_method(const optimization::problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates)
{
    const std::uint32_t cap = C == newton_cap::handed ? max_iterations : 20u;
    Eigen::VectorXd x       = x0;
    iterates.push_back(x);
    expected<Eigen::VectorXd, refusal> g = p.gradient(x);
    for(std::uint32_t i = 0; g && !p.stationary(*g) && i < cap; ++i)
    {
        const expected<Eigen::MatrixXd, refusal> h = p.hessian(x);
        if(!h)
            return praxis::unexpected(h.error());
        x -= newton_step_along<S>(*h, *g);
        iterates.push_back(x);
        g = p.gradient(x);
    }
    if(!g)
        return praxis::unexpected(g.error());
    if(!p.stationary(*g))
        return praxis::unexpected(refusal::no_solution);
    return x;
}

template<descent_cap C>
expected<Eigen::VectorXd, refusal> steepest_descent(const optimization::problem &p, const Eigen::VectorXd &x0, double step_length, std::uint32_t max_iterations,
                                                    std::vector<Eigen::VectorXd> &iterates)
{
    const std::uint32_t cap = C == descent_cap::handed ? max_iterations : 1000u;
    Eigen::VectorXd x       = x0;
    iterates.push_back(x);
    expected<Eigen::VectorXd, refusal> g = p.gradient(x);
    for(std::uint32_t i = 0; g && !p.stationary(*g) && i < cap; ++i)
    {
        x -= step_length * *g;
        iterates.push_back(x);
        g = p.gradient(x);
    }
    if(!g)
        return praxis::unexpected(g.error());
    if(!p.stationary(*g))
        return praxis::unexpected(refusal::no_solution);
    return x;
}

using newton_slot  = decltype(optimization::minimizer_ops::newton);
using descent_slot = decltype(optimization::minimizer_ops::gradient_descent);

struct literal_newton
{
    newton_step stepping;
    newton_cap cap;
    newton_slot newton;
};

struct literal_descent
{
    descent_cap cap;
    descent_slot gradient_descent;
};

inline constexpr std::array<literal_newton, 10> literal_newtons{
        literal_newton{newton_step::inverse, newton_cap::handed, &newton_method<newton_step::inverse, newton_cap::handed>},
        literal_newton{newton_step::inverse, newton_cap::twenty, &newton_method<newton_step::inverse, newton_cap::twenty>},
        literal_newton{newton_step::ldlt, newton_cap::handed, &newton_method<newton_step::ldlt, newton_cap::handed>},
        literal_newton{newton_step::ldlt, newton_cap::twenty, &newton_method<newton_step::ldlt, newton_cap::twenty>},
        literal_newton{newton_step::partial_lu, newton_cap::handed, &newton_method<newton_step::partial_lu, newton_cap::handed>},
        literal_newton{newton_step::partial_lu, newton_cap::twenty, &newton_method<newton_step::partial_lu, newton_cap::twenty>},
        literal_newton{newton_step::cod, newton_cap::handed, &newton_method<newton_step::cod, newton_cap::handed>},
        literal_newton{newton_step::cod, newton_cap::twenty, &newton_method<newton_step::cod, newton_cap::twenty>},
        literal_newton{newton_step::householder_qr, newton_cap::handed, &newton_method<newton_step::householder_qr, newton_cap::handed>},
        literal_newton{newton_step::householder_qr, newton_cap::twenty, &newton_method<newton_step::householder_qr, newton_cap::twenty>},
};

inline constexpr std::array<literal_descent, 2> literal_descents{
        literal_descent{descent_cap::handed, &steepest_descent<descent_cap::handed>},
        literal_descent{descent_cap::thousand, &steepest_descent<descent_cap::thousand>},
};

inline std::string newton_label(const literal_newton &form)
{
    constexpr std::array<std::string_view, 5> steps{"inverse", "ldlt", "partial_lu", "cod", "householder_qr"};
    constexpr std::array<std::string_view, 2> caps{"handed", "twenty"};
    return std::string(steps.at(static_cast<std::size_t>(form.stepping))) + "." + std::string(caps.at(static_cast<std::size_t>(form.cap)));
}

inline std::string descent_label(const literal_descent &form)
{
    constexpr std::array<std::string_view, 2> caps{"handed", "thousand"};
    return "descent." + std::string(caps.at(static_cast<std::size_t>(form.cap)));
}

}

#endif
