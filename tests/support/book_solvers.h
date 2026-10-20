#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_SOLVERS_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_SOLVERS_H

#include "praxis/optimization/solver.h"
#include "praxis/optimization/problem.h"

#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>

#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string_view>

// Lynch & Park, Modern Robotics, sec. 6.2.1: Newton-Raphson, x_{k+1} = x_k + J^+(x_k) e(x_k), iterated
// until the error converges or the cap is reached.
namespace praxis::tests::book {

enum class solver_step : std::uint8_t
{
    cod,
    svd,
    normal
};

enum class solver_cap : std::uint8_t
{
    handed,
    twenty
};

// The full-rank pseudoinverse of a wide or square J written out: J^T (J J^T)^-1.
template<solver_step S>
Eigen::VectorXd solver_step_along(const Eigen::MatrixXd &j, const Eigen::VectorXd &e)
{
    if constexpr(S == solver_step::cod)
        return Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>(j).solve(e);
    else if constexpr(S == solver_step::svd)
        return Eigen::JacobiSVD<Eigen::MatrixXd>(j, Eigen::ComputeThinU | Eigen::ComputeThinV).solve(e);
    else
        return j.transpose() * (j * j.transpose()).inverse() * e;
}

template<solver_step S, solver_cap C>
expected<Eigen::VectorXd, refusal> newton_raphson(const optimization::problem &p, const Eigen::VectorXd &x0, std::uint32_t max_iterations, std::vector<Eigen::VectorXd> &iterates)
{
    const std::uint32_t cap = C == solver_cap::handed ? max_iterations : 20u;
    Eigen::VectorXd x       = x0;
    iterates.push_back(x);
    expected<Eigen::VectorXd, refusal> e = p.error(x);
    for(std::uint32_t i = 0; e && !p.converged(*e) && i < cap; ++i)
    {
        const expected<Eigen::MatrixXd, refusal> j = p.jacobian(x);
        if(!j)
            return praxis::unexpected(j.error());
        x += solver_step_along<S>(*j, *e);
        iterates.push_back(x);
        e = p.error(x);
    }
    if(!e)
        return praxis::unexpected(e.error());
    if(!p.converged(*e))
        return praxis::unexpected(refusal::no_solution);
    return x;
}

using solve_slot = decltype(optimization::solver_ops::solve);

struct literal_solver
{
    solver_step stepping;
    solver_cap cap;
    solve_slot solve;
};

inline constexpr std::array<literal_solver, 6> literal_solvers{
        literal_solver{solver_step::cod, solver_cap::handed, &newton_raphson<solver_step::cod, solver_cap::handed>},
        literal_solver{solver_step::cod, solver_cap::twenty, &newton_raphson<solver_step::cod, solver_cap::twenty>},
        literal_solver{solver_step::svd, solver_cap::handed, &newton_raphson<solver_step::svd, solver_cap::handed>},
        literal_solver{solver_step::svd, solver_cap::twenty, &newton_raphson<solver_step::svd, solver_cap::twenty>},
        literal_solver{solver_step::normal, solver_cap::handed, &newton_raphson<solver_step::normal, solver_cap::handed>},
        literal_solver{solver_step::normal, solver_cap::twenty, &newton_raphson<solver_step::normal, solver_cap::twenty>},
};

inline std::string solver_label(const literal_solver &form)
{
    constexpr std::array<std::string_view, 3> steps{"cod", "svd", "normal"};
    constexpr std::array<std::string_view, 2> caps{"handed", "twenty"};
    return std::string(steps.at(static_cast<std::size_t>(form.stepping))) + "." + std::string(caps.at(static_cast<std::size_t>(form.cap)));
}

}

#endif
