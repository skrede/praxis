#include "praxis/optimization.h"
#include "praxis/optimization/baseline/solver.h"

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

namespace {

// Every contract the module publishes, named in a translation unit that includes this extension's
// headers and nothing else: a contract that stopped being reachable fails to compile here.
struct reachable_types
{
    praxis::optimization::box region;
    praxis::optimization::problem posed;
    praxis::optimization::solver_ops solver;
    praxis::optimization::solver_slot_set expected;
    praxis::optimization::capabilities composed;
};

}

namespace praxis::optimization::probe {

// A designated initializer naming only the members a residual solve reads; the target compiles with
// warnings as errors, so an omitted member lacking an initializer fails here.
problem residual_problem()
{
    return problem{
            .error     = [](const Eigen::VectorXd &x) -> expected<Eigen::VectorXd, refusal> { return Eigen::VectorXd(-x); },
            .jacobian  = [](const Eigen::VectorXd &x) -> expected<Eigen::MatrixXd, refusal> { return Eigen::MatrixXd::Identity(x.size(), x.size()); },
            .converged = [](const Eigen::VectorXd &e) -> bool { return e.norm() < 1.0; },
    };
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

    const expected<Eigen::VectorXd, refusal> root          = types.solver.solve(types.posed, Eigen::VectorXd::Ones(1), 4u, iterates);
    const expected<Eigen::VectorXd, refusal> refused       = inert::solve(types.posed, Eigen::VectorXd::Ones(1), 4u, iterates);
    const std::array<capability_view, 1> reported          = capability_views(types.composed);
    const expected<Eigen::VectorXd, refusal> reference_run = solve(types.posed, Eigen::VectorXd::Ones(1), 4u, iterates);

    return count_defaults(reported) + iterates.size() + static_cast<std::size_t>(types.region.lower.size()) + (types.expected.contains(solver_slot::solve) ? 1u : 0u) +
            (root ? 1u : 0u) + (refused ? 1u : 0u) + (reference_run ? 1u : 0u) + (holds_default(view_of(types.solver), 0) ? 1u : 0u);
}

}
