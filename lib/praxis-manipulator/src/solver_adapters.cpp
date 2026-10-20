#include "praxis/manipulator/solver_adapters.h"

#include "iterated_solve.h"

#include "praxis/optimization/problem.h"

#include "praxis/evaluation/tolerance.h"

#include <spdlog/spdlog.h>

#include <Eigen/Core>

#include <limits>
#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <optional>
#include <algorithm>

namespace praxis::manipulator {

namespace {

struct arm_residual
{
    transform desired;
    screw_chain chain;
    rigid_motion::screw_ops screw;
    forward_kinematics_ops forward;
    rigid_motion::frame_ops frames;
    differential_kinematics_ops differential;
};

using residual_map = expected<twist, refusal> (*)(const arm_residual &arm, const joint_vector &theta);
using columns_map  = expected<jacobian, refusal> (*)(const arm_residual &arm, const joint_vector &theta);

expected<transform, refusal> reached(const arm_residual &arm, const joint_vector &theta)
{
    return arm.forward.forward_kinematics(arm.screw, arm.chain.home, arm.chain.space_screws, theta);
}

expected<twist, refusal> body_frame_error(const arm_residual &arm, const joint_vector &theta)
{
    const expected<transform, refusal> tsb = reached(arm, theta);
    if(!tsb)
        return unexpected(tsb.error());

    return body_twist(arm.screw, arm.frames, *tsb, arm.desired);
}

// V_s = [Ad_T_sb] V_b.
expected<twist, refusal> space_frame_error(const arm_residual &arm, const joint_vector &theta)
{
    const expected<transform, refusal> tsb = reached(arm, theta);
    if(!tsb)
        return unexpected(tsb.error());
    const expected<twist, refusal> vb = body_twist(arm.screw, arm.frames, *tsb, arm.desired);
    if(!vb)
        return vb;
    const expected<adjoint, refusal> ad = arm.screw.adjoint_matrix_from_transform(*tsb);
    if(!ad)
        return unexpected(ad.error());

    return twist(*ad * *vb);
}

expected<jacobian, refusal> body_frame_jacobian(const arm_residual &arm, const joint_vector &theta)
{
    return arm.differential.body_jacobian(arm.screw, arm.frames, arm.forward, arm.chain.home, arm.chain.space_screws, theta);
}

expected<jacobian, refusal> space_frame_jacobian(const arm_residual &arm, const joint_vector &theta)
{
    return arm.differential.space_jacobian(arm.screw, arm.chain.space_screws, theta);
}

template<typename Dynamic, typename Fixed>
expected<Dynamic, refusal> finite(const expected<Fixed, refusal> &value)
{
    if(!value)
        return unexpected(value.error());
    if(!value->allFinite())
        return unexpected(refusal::degenerate);

    return Dynamic(*value);
}

// A Jacobian of another width than the iterate cannot be stepped through.
expected<Eigen::MatrixXd, refusal> columns_at(const arm_residual &arm, columns_map columns, const Eigen::VectorXd &x)
{
    expected<Eigen::MatrixXd, refusal> j = finite<Eigen::MatrixXd>(columns(arm, x));
    if(j && j->cols() != x.size())
        return unexpected(refusal::degenerate);

    return j;
}

bool within(const Eigen::VectorXd &error, const solver_parameters &parameters)
{
    return error.size() == 6 && error.head<3>().norm() <= parameters.orientation_tol && error.tail<3>().norm() <= parameters.position_tol;
}

// Lynch & Park, Modern Robotics, Def. 3.24: a unit screw of pitch w . v = 0 is a pure rotation, so a
// whole turn about it is no displacement. Any other screw answers zero.
double whole_turn(const screw_axis &s)
{
    const bool rotation = is_approx_equal(s.head<3>().norm(), 1.0) && is_approx_equal(s.head<3>().dot(s.tail<3>()), 0.0);

    return rotation ? 2.0 * std::numbers::pi : 0.0;
}

// A joint the limits do not cover is free on both sides; one that turns whole wraps by its turn.
optimization::box joint_box(const screw_chain &chain)
{
    constexpr double unbounded = std::numeric_limits<double>::infinity();
    const joint_limits &limits = chain.limits;
    const auto width           = static_cast<Eigen::Index>(chain.joint_count());
    const Eigen::Index covered = std::min({width, limits.lower_position.size(), limits.upper_position.size()});

    optimization::box bounds{.lower = Eigen::VectorXd::Constant(width, -unbounded), .upper = Eigen::VectorXd::Constant(width, unbounded), .period = Eigen::VectorXd(width)};
    bounds.lower.head(covered) = limits.lower_position.head(covered);
    bounds.upper.head(covered) = limits.upper_position.head(covered);
    for(Eigen::Index joint = 0; joint < width; ++joint)
        bounds.period[joint] = whole_turn(chain.space_screws[static_cast<std::size_t>(joint)]);

    return bounds;
}

// The problem holds what it reads, so a solver may keep it past the call.
optimization::problem residual_problem(const std::shared_ptr<const arm_residual> &arm, residual_map error, columns_map columns, const solver_parameters &parameters)
{
    return optimization::problem{
            .error     = [arm, error](const Eigen::VectorXd &x) { return finite<Eigen::VectorXd>(error(*arm, x)); },
            .jacobian  = [arm, columns](const Eigen::VectorXd &x) { return columns_at(*arm, columns, x); },
            .converged = [parameters](const Eigen::VectorXd &e) { return within(e, parameters); },
            .bounds    = joint_box(arm->chain),
    };
}

// x_0 has no step behind it, and both halves are NaN at an iterate where the error cannot be read.
void mapped(const optimization::problem &p, const std::vector<Eigen::VectorXd> &iterates, ik_result &answer)
{
    constexpr double unread = std::numeric_limits<double>::quiet_NaN();

    for(std::size_t k = 0; k < iterates.size(); ++k)
    {
        const joint_vector &x                      = iterates[k];
        const expected<Eigen::VectorXd, refusal> e = p.error(x);
        const double step                          = k == 0 ? 0.0 : (x.size() == iterates[k - 1].size() ? (x - iterates[k - 1]).norm() : unread);
        answer.iterations.push_back(iteration_state{x, e ? e->head<3>().norm() : unread, e ? e->tail<3>().norm() : unread, step, static_cast<std::uint32_t>(k)});
    }
}

// A solve that did not converge is no_solution at the slot; one that could not be run passes as it came.
refusal answered(const optimization::solver_ops &solver, refusal refused)
{
    if(refused == refusal::not_implemented && solver.solve == &optimization::inert::solve)
        spdlog::warn("praxis: 'solver.solve' is not implemented, so the inverse kinematics handed it answers no configuration");

    return refused == refusal::not_implemented || refused == refusal::unsupported_input ? refused : refusal::no_solution;
}

// The solver's word that it converged is not taken: the problem is asked again at the root.
bool converged_at(const optimization::problem &p, const Eigen::VectorXd &root, Eigen::Index width)
{
    if(root.size() != width || !root.allFinite())
        return false;

    const expected<Eigen::VectorXd, refusal> e = p.error(root);
    return e && p.converged(*e);
}

expected<void, refusal> solved(const std::shared_ptr<const arm_residual> &arm, residual_map error, columns_map columns, const optimization::solver_ops &solver, const joint_vector &j0,
                               const solver_parameters &parameters, ik_result &answer)
{
    if(const std::optional<refusal> refused = refused_at_entry(arm->frames, arm->chain, arm->desired, j0))
        return unexpected(*refused);

    const optimization::problem p = residual_problem(arm, error, columns, parameters);
    std::vector<Eigen::VectorXd> iterates;
    const expected<Eigen::VectorXd, refusal> root = solver.solve(p, j0, parameters.max_iterations_per_attempt, iterates);
    mapped(p, iterates, answer);
    if(!root)
        return unexpected(answered(solver, root.error()));
    if(!converged_at(p, *root, j0.size()))
        return unexpected(refusal::no_solution);

    answer.solutions.push_back(*root);

    return {};
}

std::shared_ptr<const arm_residual> held(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const forward_kinematics_ops &forward,
                                         const differential_kinematics_ops &differential, const screw_chain &chain, const transform &desired)
{
    return std::make_shared<const arm_residual>(arm_residual{desired, chain, screw, forward, frames, differential});
}

}

expected<void, refusal> body_inverse_kinematics(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const forward_kinematics_ops &forward,
                                                const differential_kinematics_ops &differential, const optimization::solver_ops &solver, const screw_chain &chain,
                                                const transform &desired, const joint_vector &j0, const solver_parameters &parameters, ik_result &answer)
{
    return solved(held(screw, frames, forward, differential, chain, desired), &body_frame_error, &body_frame_jacobian, solver, j0, parameters, answer);
}

expected<void, refusal> space_inverse_kinematics(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const forward_kinematics_ops &forward,
                                                 const differential_kinematics_ops &differential, const optimization::solver_ops &solver, const screw_chain &chain,
                                                 const transform &desired, const joint_vector &j0, const solver_parameters &parameters, ik_result &answer)
{
    return solved(held(screw, frames, forward, differential, chain, desired), &space_frame_error, &space_frame_jacobian, solver, j0, parameters, answer);
}

}
