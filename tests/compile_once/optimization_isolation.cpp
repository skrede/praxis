#include "praxis/optimization.h"
#include "praxis/optimization/baseline/solver.h"
#include "praxis/optimization/baseline/minimizer.h"

#if defined(HPP_GUARD_MEIOS_URDF_LOAD_H) || defined(SPDLOG_VER_MAJOR)
    #error "the optimization umbrella pulled in a meios or spdlog header"
#endif
#if __has_include(<meios/urdf/load.h>)
    #error "a meios include directory reached a translation unit linking only the optimization module"
#endif
#if __has_include("praxis/rigid_motion/types.h") || __has_include("praxis/trajectory/types.h") || __has_include("praxis/manipulator/types.h")
    #error "another extension's include directory reached a translation unit linking only the optimization module"
#endif

#include <Eigen/Core>

#include <array>
#include <vector>
#include <cstddef>
#include <optional>

namespace {

// Every contract the module publishes, named in a translation unit that includes this extension's
// headers and nothing else: a contract that stopped being reachable fails to compile here.
struct reachable_types
{
    praxis::optimization::box region;
    praxis::optimization::problem posed;
    praxis::optimization::solver_ops solver;
    praxis::optimization::solver_slot_set expected;
    praxis::optimization::minimizer_ops minimizer;
    praxis::optimization::minimizer_slot_set expected_minimizers;
    praxis::optimization::capabilities composed;
    std::optional<praxis::optimization::registered_curve> curve;
    praxis::optimization::scenario_snapshot published;
    praxis::optimization::scenario_composition scenario;
};

}

namespace praxis::optimization::probe {

// Designated initializers naming only the members a bounded residual solve reads; the target compiles
// with warnings as errors, so an omitted member lacking an initializer fails here.
problem residual_problem()
{
    return problem{
            .error     = [](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal> { return Eigen::VectorXd(-x); },
            .jacobian  = [](const Eigen::VectorXd &x) -> expected<Eigen::MatrixXd, refusal> { return Eigen::MatrixXd::Identity(x.size(), x.size()); },
            .converged = [](const Eigen::VectorXd &e) -> bool { return e.norm() < 1.0; },
            .bounds    = box{.lower = -Eigen::VectorXd::Ones(1), .upper = Eigen::VectorXd::Ones(1)},
    };
}

// Designated initializers naming only the objective form, every residual member and the bounds
// omitted.
problem objective_problem()
{
    return problem{
            .objective  = [](const Eigen::VectorXd &x) -> expected<double, refusal> { return 0.5 * x.squaredNorm(); },
            .gradient   = [](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal> { return x; },
            .hessian    = [](const Eigen::VectorXd &x) -> expected<Eigen::MatrixXd, refusal> { return Eigen::MatrixXd::Identity(x.size(), x.size()); },
            .stationary = [](const Eigen::VectorXd &g) -> bool { return g.norm() < 1.0; },
    };
}

std::size_t minimization_surface(const capabilities &composed, std::vector<Eigen::VectorXd> &iterates)
{
    const problem posed                              = objective_problem();
    const minimizer_ops minimizer                    = composed.minimizer;
    const expected<Eigen::VectorXd, refusal> stepped = minimizer.newton(posed, Eigen::VectorXd::Ones(1), 4u, iterates);
    const expected<Eigen::VectorXd, refusal> walked  = minimizer.gradient_descent(posed, Eigen::VectorXd::Ones(1), 0.5, 4u, iterates);
    const expected<Eigen::VectorXd, refusal> refused = inert::newton(posed, Eigen::VectorXd::Ones(1), 4u, iterates);
    const expected<Eigen::VectorXd, refusal> idle    = inert::gradient_descent(posed, Eigen::VectorXd::Ones(1), 0.5, 4u, iterates);
    const expected<Eigen::VectorXd, refusal> direct  = gradient_descent(posed, Eigen::VectorXd::Ones(1), 0.5, 4u, iterates);
    const expected<Eigen::VectorXd, refusal> reached = newton(posed, Eigen::VectorXd::Ones(1), 4u, iterates);

    return (stepped ? 1u : 0u) + (walked ? 1u : 0u) + (refused ? 1u : 0u) + (idle ? 1u : 0u) + (direct ? 1u : 0u) + (reached ? 1u : 0u) +
            (holds_default(view_of(minimizer), 1) ? 1u : 0u);
}

std::size_t optimization_surface()
{
    reachable_types types{};
    std::vector<Eigen::VectorXd> iterates;
    types.posed    = residual_problem();
    types.region   = box{Eigen::VectorXd::Zero(1), Eigen::VectorXd::Ones(1)};
    types.composed = baseline();
    types.solver   = types.composed.solver;
    types.expected.set(solver_slot::solve);
    types.minimizer = types.composed.minimizer;
    types.expected_minimizers.set(minimizer_slot::gradient_descent);
    types.curve.emplace("line", [](double x) { return x; }, [](double) { return 1.0; });

    const expected<Eigen::VectorXd, refusal> root          = types.solver.solve(types.posed, Eigen::VectorXd::Ones(1), 4u, iterates);
    const expected<Eigen::VectorXd, refusal> refused       = inert::solve(types.posed, Eigen::VectorXd::Ones(1), 4u, iterates);
    const std::array<capability_view, 2> reported          = capability_views(types.composed);
    const expected<Eigen::VectorXd, refusal> reference_run = solve(types.posed, Eigen::VectorXd::Ones(1), 4u, iterates);

    return count_defaults(reported) + iterates.size() + static_cast<std::size_t>(types.region.lower.size()) + (types.expected.contains(solver_slot::solve) ? 1u : 0u) +
            (root ? 1u : 0u) + (refused ? 1u : 0u) + (reference_run ? 1u : 0u) + (holds_default(view_of(types.solver), 0) ? 1u : 0u) +
            (curve_problem(*types.curve, 1.0e-6).bounds ? 1u : 0u) + types.published.abscissa.size() + (types.scenario.checks_derivative ? 1u : 0u) +
            minimization_surface(types.composed, iterates) + (types.expected_minimizers.contains(minimizer_slot::newton) ? 1u : 0u) +
            (holds_default(view_of(types.minimizer), 0) ? 1u : 0u);
}

}
