#include "praxis/manipulator/kinematics.h"

#include <span>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <optional>
#include <functional>

namespace praxis::manipulator {

namespace {

// Every member of the limits carries one entry per degree of freedom, and the controls that read
// them index all four by joint with no bound of their own.
bool limits_cover_every_joint(const screw_chain &chain)
{
    const Eigen::Index count = static_cast<Eigen::Index>(chain.joint_count());

    return chain.limits.velocity.size() == count && chain.limits.acceleration.size() == count && chain.limits.lower_position.size() == count &&
            chain.limits.upper_position.size() == count;
}

}

solver_parameters::solver_parameters()
        : position_tol(1.e-7)
        , orientation_tol(1.e-7)
        , max_iterations_per_attempt(70u)
{
}

solver_parameters::solver_parameters(double position_tolerance, double orientation_tolerance, std::uint32_t iteration_limit)
        : position_tol(position_tolerance)
        , orientation_tol(orientation_tolerance)
        , max_iterations_per_attempt(iteration_limit)
{
}

kinematics::kinematics()
        : kinematics(screw_chain(), forward_kinematics_ops(), differential_kinematics_ops(), inverse_kinematics_ops(), rigid_motion::frame_ops())
{
}

kinematics::kinematics(screw_chain chain, forward_kinematics_ops forward, differential_kinematics_ops differential, inverse_kinematics_ops inverse,
                       const rigid_motion::frame_ops &frames)
        : m_space(std::move(chain))
        , m_body()
        , m_body_unavailable()
        , m_fk(forward)
        , m_dk(differential)
        , m_ik(inverse)
        , m_frames(frames)
        , m_last()
        , m_solve_tally(0u)
{
}

// The body chain is derived here rather than on demand, so nothing a const answer reads is written
// later. A derivation that refuses is recorded and carried to the body Jacobian instead of stopping
// the composition: forward kinematics needs no body screws, so requiring the derivation to compose
// at all would make one capability the price of another. An empty span of space screws has no
// adjoint to apply, so the derivation is skipped rather than asked for and the body chain is left as
// empty as the space chain it would have been taken from.
expected<kinematics, refusal> kinematics::compose(screw_chain chain, forward_kinematics_ops forward, differential_kinematics_ops differential, inverse_kinematics_ops inverse,
                                                  const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames)
{
    kinematics composed(std::move(chain), forward, differential, inverse, frames);
    if(!limits_cover_every_joint(composed.m_space))
        return unexpected(refusal::unsupported_input);
    if(composed.m_space.space_screws.empty())
        return composed;

    expected<std::vector<screw_axis>, refusal> body = forward.body_screws_from_space(screw, frames, composed.m_space.home, composed.m_space.space_screws);
    if(!body)
        composed.m_body_unavailable = body.error();
    else
        composed.m_body = screw_chain(composed.m_space.home, std::move(*body), composed.m_space.limits);

    return composed;
}

std::uint32_t kinematics::joint_count() const
{
    return static_cast<std::uint32_t>(m_space.joint_count());
}

const screw_chain &kinematics::space_chain() const
{
    return m_space;
}

expected<std::reference_wrapper<const screw_chain>, refusal> kinematics::body_chain() const
{
    if(m_body_unavailable.has_value())
        return unexpected(*m_body_unavailable);

    return std::cref(m_body);
}

expected<transform, refusal> kinematics::fk_solve(const joint_vector &joint_positions) const
{
    return m_fk.forward_kinematics(m_space.home, m_space.space_screws, joint_positions);
}

expected<transform, refusal> kinematics::body_fk_solve(const joint_vector &joint_positions) const
{
    if(m_body_unavailable.has_value())
        return unexpected(*m_body_unavailable);

    return m_fk.body_forward_kinematics(m_frames, m_space.home, m_body.space_screws, joint_positions);
}

expected<jacobian, refusal> kinematics::space_jacobian(const joint_vector &joint_positions) const
{
    return m_dk.space_jacobian(m_space.space_screws, joint_positions);
}

expected<jacobian, refusal> kinematics::body_jacobian(const joint_vector &joint_positions) const
{
    if(m_body_unavailable.has_value())
        return unexpected(*m_body_unavailable);

    return m_dk.body_jacobian(m_body.space_screws, joint_positions);
}

expected<joint_vector, refusal> kinematics::ik_solve(const transform &desired_pose, const joint_vector &j0, const solver_parameters &parameters) const
{
    const expected<void, refusal> solved = solve(desired_pose, j0, parameters);
    if(!solved)
        return unexpected(solved.error());
    if(m_last.solutions.empty())
        return unexpected(refusal::no_solution);

    return m_last.solutions.front();
}

expected<joint_vector, refusal> kinematics::ik_solve(const transform &desired_pose, const joint_vector &j0, const solver_parameters &parameters,
                                                     const std::function<std::optional<std::size_t>(std::span<const joint_vector>)> &solution_selector) const
{
    const expected<void, refusal> solved = solve(desired_pose, j0, parameters);
    if(!solved)
        return unexpected(solved.error());
    if(m_last.solutions.empty())
        return unexpected(refusal::no_solution);

    const std::optional<std::size_t> chosen = solution_selector(m_last.solutions);
    if(!chosen.has_value())
        return unexpected(refusal::no_solution);
    if(*chosen >= m_last.solutions.size())
        return unexpected(refusal::degenerate);

    return m_last.solutions[*chosen];
}

expected<std::span<const joint_vector>, refusal> kinematics::configurations_reaching(const transform &desired_pose) const
{
    m_last.solutions.clear();
    m_last.iterations.clear();
    ++m_solve_tally;

    const expected<void, refusal> answered = m_ik.analytic_inverse_kinematics(m_fk, m_space, desired_pose, m_last);
    if(!answered)
        return unexpected(answered.error());

    return std::span<const joint_vector>(m_last.solutions);
}

std::span<const joint_vector> kinematics::solutions() const
{
    return m_last.solutions;
}

std::span<const iteration_state> kinematics::iterations() const
{
    return m_last.iterations;
}

std::uint64_t kinematics::solve_count() const
{
    return m_solve_tally;
}

// The buffer the answer is recorded into -- and with it the span iterations() hands out -- lasts only
// until the next solve here.
expected<void, refusal> kinematics::solve(const transform &desired_pose, const joint_vector &j0, const solver_parameters &parameters) const
{
    m_last.solutions.clear();
    m_last.iterations.clear();
    ++m_solve_tally;

    return m_ik.inverse_kinematics(m_fk, m_dk, m_space, desired_pose, j0, parameters, m_last);
}

}
