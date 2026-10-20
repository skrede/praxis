#include "evaluation_tables.h"

#include "praxis/optimization/slots.h"
#include "praxis/optimization/problem.h"

#include "praxis/evaluation/residual.h"

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <limits>
#include <vector>
#include <cstddef>
#include <algorithm>

namespace praxis::optimization {

namespace {

const solver_ops &solvers_of(const void *value)
{
    return *static_cast<const solver_ops *>(value);
}

// g(x) = x^3 - 2x - 5
Eigen::VectorXd polynomial(const Eigen::VectorXd &x)
{
    return Eigen::VectorXd::Constant(1, x[0] * x[0] * x[0] - 2.0 * x[0] - 5.0);
}

Eigen::MatrixXd polynomial_slope(const Eigen::VectorXd &x)
{
    return Eigen::MatrixXd::Constant(1, 1, 3.0 * x[0] * x[0] - 2.0);
}

// g(x) = cos(x) - x
Eigen::VectorXd transcendental(const Eigen::VectorXd &x)
{
    return Eigen::VectorXd::Constant(1, std::cos(x[0]) - x[0]);
}

Eigen::MatrixXd transcendental_slope(const Eigen::VectorXd &x)
{
    return Eigen::MatrixXd::Constant(1, 1, -std::sin(x[0]) - 1.0);
}

// g(x, y) = (x^2 + y^2 - 4, xy - 1)
Eigen::VectorXd square_system(const Eigen::VectorXd &x)
{
    return Eigen::Vector2d(x[0] * x[0] + x[1] * x[1] - 4.0, x[0] * x[1] - 1.0);
}

Eigen::MatrixXd square_system_slope(const Eigen::VectorXd &x)
{
    return Eigen::Matrix2d{{2.0 * x[0], 2.0 * x[1]}, {x[1], x[0]}};
}

// g(x, y) = x^2 + y^2 - 1
Eigen::VectorXd underdetermined(const Eigen::VectorXd &x)
{
    return Eigen::VectorXd::Constant(1, x[0] * x[0] + x[1] * x[1] - 1.0);
}

Eigen::MatrixXd underdetermined_slope(const Eigen::VectorXd &x)
{
    return Eigen::RowVector2d(2.0 * x[0], 2.0 * x[1]);
}

struct posed_problem
{
    Eigen::VectorXd (*g)(const Eigen::VectorXd &x);
    Eigen::MatrixXd (*slope)(const Eigen::VectorXd &x);
    Eigen::Index dimension;
    double seed_half_width;
};

constexpr std::array<posed_problem, 4> posed{
        posed_problem{&polynomial, &polynomial_slope, 1, polynomial_seed_half_width},
        posed_problem{&transcendental, &transcendental_slope, 1, transcendental_seed_half_width},
        posed_problem{&square_system, &square_system_slope, 2, square_system_seed_half_width},
        posed_problem{&underdetermined, &underdetermined_slope, 2, underdetermined_seed_half_width},
};

// Lynch & Park, Modern Robotics, sec. 6.2 with x_d = 0: e(x) = -g(x) and J = dg/dx.
problem book_form(const posed_problem &p)
{
    return problem{
            .error = [g = p.g](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal>
            {
                const Eigen::VectorXd e = -g(x);
                if(!e.allFinite())
                    return praxis::unexpected(refusal::degenerate);
                return e;
            },
            .jacobian = [slope = p.slope](const Eigen::VectorXd &x) -> expected<Eigen::MatrixXd, refusal>
            {
                const Eigen::MatrixXd j = slope(x);
                if(!j.allFinite())
                    return praxis::unexpected(refusal::degenerate);
                return j;
            },
            .converged = [](const Eigen::VectorXd &e) -> bool { return e.lpNorm<Eigen::Infinity>() <= solver_row_tolerance; },
    };
}

const std::array<problem, 4> &problems()
{
    static const std::array<problem, 4> posed_in_book_form{book_form(posed[0]), book_form(posed[1]), book_form(posed[2]), book_form(posed[3])};

    return posed_in_book_form;
}

constexpr evaluation::residual_kind answer_kind = evaluation::residual_kind::element_wise;

// The greatest element of the problem's own error at the answer: unbounded where the answer has
// another dimension than the problem or the error there is refused.
evaluation::residual error_at(const problem &p, Eigen::Index dimension, const Eigen::VectorXd &answer)
{
    constexpr double unbounded = std::numeric_limits<double>::infinity();
    if(answer.size() != dimension)
        return evaluation::residual{answer_kind, unbounded, 0.0};

    const expected<Eigen::VectorXd, refusal> e = p.error(answer);

    return evaluation::residual{answer_kind, e ? e->lpNorm<Eigen::Infinity>() : unbounded, 0.0};
}

evaluation::case_result compare_solve(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const auto drawn_index    = static_cast<std::size_t>(drawn.uniform(0.0, static_cast<double>(posed.size())));
    const std::size_t index   = std::min(drawn_index, posed.size() - 1u);
    const posed_problem &pose = posed[index];

    Eigen::VectorXd x0(pose.dimension);
    for(Eigen::Index i = 0; i < x0.size(); ++i)
        x0[i] = drawn.uniform(-pose.seed_half_width, pose.seed_half_width);

    const problem &p = problems()[index];
    std::vector<Eigen::VectorXd> first_iterates;
    std::vector<Eigen::VectorXd> second_iterates;
    const expected<Eigen::VectorXd, refusal> held    = solvers_of(first).solve(p, x0, solver_row_iteration_limit, first_iterates);
    const expected<Eigen::VectorXd, refusal> against = solvers_of(second).solve(p, x0, solver_row_iteration_limit, second_iterates);

    return evaluation::agreed_or_refused(held, against, [&](const Eigen::VectorXd &answer, const Eigen::VectorXd &) { return error_at(p, pose.dimension, answer); }, allowed);
}

// The one row is in the enumerator order of solver_slot, its name spelled exactly as the descriptor
// table spells it.
constexpr evaluation::tolerance_pair answer_allowance{solver_row_tolerance, solver_row_tolerance};

constexpr std::array solver_table{
        evaluation::slot_evaluation{"solver.solve", answer_kind, answer_allowance, &compare_solve},
};

static_assert(solver_table.size() == static_cast<std::size_t>(solver_slot::count));

constexpr evaluation::capability_evaluations<solver_ops> evaluated_solvers{"optimization", solver_table};

}

const evaluation::capability_evaluations<solver_ops> &solver_evaluations()
{
    return evaluated_solvers;
}

}
