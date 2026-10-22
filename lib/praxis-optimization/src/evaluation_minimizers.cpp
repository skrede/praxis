#include "evaluation_tables.h"

#include "praxis/optimization/slots.h"
#include "praxis/optimization/problem.h"
#include "praxis/optimization/minimizer.h"

#include "praxis/evaluation/residual.h"

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <limits>
#include <vector>
#include <cstddef>
#include <algorithm>
#include <type_traits>

namespace praxis::optimization {

namespace {

struct objective_at
{
    double value;
    Eigen::VectorXd gradient;
    Eigen::MatrixXd hessian;
};

// f(x) = x^4 - 3x^2 + x
objective_at quartic(const Eigen::VectorXd &x)
{
    const double t = x[0];
    return {t * t * t * t - 3.0 * t * t + t, Eigen::VectorXd::Constant(1, 4.0 * t * t * t - 6.0 * t + 1.0), Eigen::MatrixXd::Constant(1, 1, 12.0 * t * t - 6.0)};
}

// f(x) = exp(x) - 2x
objective_at exponential(const Eigen::VectorXd &x)
{
    const double t = x[0];
    return {std::exp(t) - 2.0 * t, Eigen::VectorXd::Constant(1, std::exp(t) - 2.0), Eigen::MatrixXd::Constant(1, 1, std::exp(t))};
}

// f(x, y) = (x^2 + 10 y^2) / 2
objective_at elongated_bowl(const Eigen::VectorXd &x)
{
    return {(x[0] * x[0] + 10.0 * x[1] * x[1]) / 2.0, Eigen::Vector2d(x[0], 10.0 * x[1]), Eigen::Matrix2d{{1.0, 0.0}, {0.0, 10.0}}};
}

// Nocedal & Wright, Numerical Optimization, 1st ed., eq. (2.23): f(x, y) = 100 (y - x^2)^2 + (1 - x)^2.
objective_at rosenbrock(const Eigen::VectorXd &x)
{
    const double valley = x[1] - x[0] * x[0];
    return {100.0 * valley * valley + (1.0 - x[0]) * (1.0 - x[0]), Eigen::Vector2d(-400.0 * x[0] * valley - 2.0 * (1.0 - x[0]), 200.0 * valley),
            Eigen::Matrix2d{{1200.0 * x[0] * x[0] - 400.0 * x[1] + 2.0, -400.0 * x[0]}, {-400.0 * x[0], 200.0}}};
}

// f(x) = (x - 1)^2
objective_at shifted_parabola(const Eigen::VectorXd &x)
{
    return {(x[0] - 1.0) * (x[0] - 1.0), Eigen::VectorXd::Constant(1, 2.0 * (x[0] - 1.0)), Eigen::MatrixXd::Constant(1, 1, 2.0)};
}

// f(x, y) = (x^2 + 4 y^2) / 2
objective_at oval_bowl(const Eigen::VectorXd &x)
{
    return {(x[0] * x[0] + 4.0 * x[1] * x[1]) / 2.0, Eigen::Vector2d(x[0], 4.0 * x[1]), Eigen::Matrix2d{{1.0, 0.0}, {0.0, 4.0}}};
}

// f(x, y) = x^4 / 4 - x^2 / 2 + y^2 / 2
objective_at double_well(const Eigen::VectorXd &x)
{
    const double t = x[0];
    return {t * t * t * t / 4.0 - t * t / 2.0 + x[1] * x[1] / 2.0, Eigen::Vector2d(t * t * t - t, x[1]), Eigen::Matrix2d{{3.0 * t * t - 1.0, 0.0}, {0.0, 1.0}}};
}

struct posed_objective
{
    objective_at (*at)(const Eigen::VectorXd &x);
    Eigen::Index dimension;
    double seed_half_width;
};

constexpr std::array<posed_objective, 4> posed_for_newton{
        posed_objective{&quartic, 1, quartic_seed_half_width},
        posed_objective{&exponential, 1, exponential_seed_half_width},
        posed_objective{&elongated_bowl, 2, elongated_bowl_seed_half_width},
        posed_objective{&rosenbrock, 2, rosenbrock_seed_half_width},
};

constexpr std::array<posed_objective, 3> posed_for_descent{
        posed_objective{&shifted_parabola, 1, shifted_parabola_seed_half_width},
        posed_objective{&oval_bowl, 2, oval_bowl_seed_half_width},
        posed_objective{&double_well, 2, double_well_seed_half_width},
};

constexpr std::array<double, 3> descent_step_lengths{shifted_parabola_step_length, oval_bowl_step_length, double_well_step_length};

template<typename Value>
expected<Value, refusal> finite_or_degenerate(const Value &v)
{
    bool finite = false;
    if constexpr(std::is_same_v<Value, double>)
        finite = std::isfinite(v);
    else
        finite = v.allFinite();
    if(!finite)
        return praxis::unexpected(refusal::degenerate);
    return v;
}

// Every member refuses degenerate on a non-finite evaluation.
problem objective_form(const posed_objective &posed, bool carries_hessian)
{
    problem p{
            .objective  = [at = posed.at](const Eigen::VectorXd &x) { return finite_or_degenerate(at(x).value); },
            .gradient   = [at = posed.at](const Eigen::VectorXd &x) { return finite_or_degenerate(at(x).gradient); },
            .stationary = [](const Eigen::VectorXd &g) -> bool { return g.lpNorm<Eigen::Infinity>() <= minimizer_row_tolerance; },
    };
    if(carries_hessian)
        p.hessian = [at = posed.at](const Eigen::VectorXd &x) { return finite_or_degenerate(at(x).hessian); };
    return p;
}

constexpr evaluation::residual_kind answer_kind = evaluation::residual_kind::element_wise;

// The greatest element of the problem's own gradient at the answer: unbounded where the answer has
// another dimension than the problem or the gradient there is refused.
evaluation::residual gradient_at(const problem &p, Eigen::Index dimension, const Eigen::VectorXd &answer)
{
    constexpr double unbounded = std::numeric_limits<double>::infinity();
    if(answer.size() != dimension)
        return evaluation::residual{answer_kind, unbounded, 0.0};

    const expected<Eigen::VectorXd, refusal> g = p.gradient(answer);

    return evaluation::residual{answer_kind, g ? g->lpNorm<Eigen::Infinity>() : unbounded, 0.0};
}

// Draws a problem's index and then every coordinate of its seed, and runs both sides from that seed.
template<std::size_t N, typename Minimized>
evaluation::case_result compare_drawn(const std::array<posed_objective, N> &posed, bool carries_hessian, const void *first, const void *second, evaluation::case_source &drawn,
                                      const evaluation::tolerance_pair &allowed, Minimized minimized)
{
    const auto drawn_index      = static_cast<std::size_t>(drawn.uniform(0.0, static_cast<double>(N)));
    const std::size_t index     = std::min(drawn_index, N - 1u);
    const posed_objective &pose = posed[index];

    Eigen::VectorXd x0(pose.dimension);
    for(Eigen::Index i = 0; i < x0.size(); ++i)
        x0[i] = drawn.uniform(-pose.seed_half_width, pose.seed_half_width);

    const problem p = objective_form(pose, carries_hessian);
    return evaluation::agreed_or_refused(
            minimized(*static_cast<const minimizer_ops *>(first), p, index, x0), minimized(*static_cast<const minimizer_ops *>(second), p, index, x0),
            [&](const Eigen::VectorXd &answer, const Eigen::VectorXd &) { return gradient_at(p, pose.dimension, answer); }, allowed);
}

evaluation::case_result compare_newton(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    return compare_drawn(posed_for_newton, true, first, second, drawn, allowed,
                         [](const minimizer_ops &ops, const problem &p, std::size_t, const Eigen::VectorXd &x0)
                         {
                             std::vector<Eigen::VectorXd> iterates;
                             return ops.newton(p, x0, minimizer_row_newton_limit, iterates);
                         });
}

evaluation::case_result compare_descent(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    return compare_drawn(posed_for_descent, false, first, second, drawn, allowed,
                         [](const minimizer_ops &ops, const problem &p, std::size_t index, const Eigen::VectorXd &x0)
                         {
                             std::vector<Eigen::VectorXd> iterates;
                             return ops.gradient_descent(p, x0, descent_step_lengths[index], minimizer_row_descent_limit, iterates);
                         });
}

// The rows are in the enumerator order of minimizer_slot, each name spelled exactly as the descriptor
// table spells it.
constexpr evaluation::tolerance_pair answer_allowance{minimizer_row_tolerance, minimizer_row_tolerance};

constexpr std::array minimizer_table{
        evaluation::slot_evaluation{"minimizer.newton", answer_kind, answer_allowance, &compare_newton},
        evaluation::slot_evaluation{"minimizer.gradient_descent", answer_kind, answer_allowance, &compare_descent},
};

static_assert(minimizer_table.size() == static_cast<std::size_t>(minimizer_slot::count));

constexpr evaluation::capability_evaluations<minimizer_ops> evaluated_minimizers{"optimization", minimizer_table};

}

const evaluation::capability_evaluations<minimizer_ops> &minimizer_evaluations()
{
    return evaluated_minimizers;
}

}
