#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_KINEMATICS_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_KINEMATICS_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/screw_chain.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/types.h"

#include <span>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <functional>

namespace praxis::manipulator {

// The stopping criteria of the iterative solve: the linear and the angular half of the body twist
// are each driven below their own tolerance. Lynch & Park, Modern Robotics, section 6.2.
struct solver_parameters
{
    double position_tol;
    double orientation_tol;
    std::uint32_t max_iterations_per_attempt;

    solver_parameters();
    solver_parameters(double position_tolerance, double orientation_tolerance, std::uint32_t iteration_limit);
};

// One step of the solve. The two errors are the norms of the angular and the linear half of the body
// twist, against which the two tolerances above are compared; the step norm is the distance from the
// iterate before this one.
struct iteration_state
{
    joint_vector joint_positions;
    double angular_error;
    double linear_error;
    double step_norm;
    std::uint32_t index;
};

// What a solve answers with. A solver that reports no iterations leaves that half empty, and one that
// finds nothing leaves both empty; neither is an error the caller has to distinguish from a refusal.
struct ik_result
{
    std::vector<joint_vector> solutions;
    std::vector<iteration_state> iterations;
};

}

namespace praxis::manipulator::inert {

expected<transform, refusal> forward_kinematics(const transform &m, std::span<const screw_axis> space_screws, const joint_vector &theta);
expected<jacobian, refusal> space_jacobian(std::span<const screw_axis> space_screws, const joint_vector &theta);
expected<jacobian, refusal> body_jacobian(std::span<const screw_axis> body_screws, const joint_vector &theta);
expected<std::vector<screw_axis>, refusal> body_screws_from_space(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const transform &m,
                                                                  std::span<const screw_axis> space_screws);
expected<transform, refusal> body_forward_kinematics(const rigid_motion::frame_ops &frames, const transform &m, std::span<const screw_axis> body_screws, const joint_vector &theta);

}

namespace praxis::manipulator {

// Declaration order is frozen: a designated initializer must name members in declaration order, so
// reordering a slot breaks every project that already composes this aggregate. Appending is safe.
// The two forward maps are here, one over the space screws and one over the body screws, together
// with the derivation that carries a chain from the first frame to the second: each is a function of
// a chain and a configuration alone.
struct forward_kinematics_ops
{
    expected<transform, refusal> (*forward_kinematics)(const transform &m, std::span<const screw_axis> space_screws, const joint_vector &theta) = &inert::forward_kinematics;
    expected<transform, refusal> (*body_forward_kinematics)(const rigid_motion::frame_ops &frames, const transform &m, std::span<const screw_axis> body_screws,
                                                            const joint_vector &theta)                                                          = &inert::body_forward_kinematics;
    expected<std::vector<screw_axis>, refusal> (*body_screws_from_space)(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const transform &m,
                                                                         std::span<const screw_axis> space_screws)                              = &inert::body_screws_from_space;
};

// Declaration order is frozen: a designated initializer must name members in declaration order, so
// reordering a slot breaks every project that already composes this aggregate. Appending is safe.
// The two Jacobians are here: each maps joint rates to a twist, one expressed in the space frame and
// one in the body frame, and each is taken over the screws it is handed and the configuration alone.
struct differential_kinematics_ops
{
    expected<jacobian, refusal> (*space_jacobian)(std::span<const screw_axis> space_screws, const joint_vector &theta) = &inert::space_jacobian;
    expected<jacobian, refusal> (*body_jacobian)(std::span<const screw_axis> body_screws, const joint_vector &theta)   = &inert::body_jacobian;
};

}

namespace praxis::manipulator::inert {

expected<void, refusal> inverse_kinematics(const forward_kinematics_ops &forward, const differential_kinematics_ops &differential, const screw_chain &chain, const transform &desired,
                                           const joint_vector &j0, const solver_parameters &parameters, ik_result &answer);
expected<void, refusal> analytic_inverse_kinematics(const forward_kinematics_ops &forward, const screw_chain &chain, const transform &desired, ik_result &answer);

}

namespace praxis::manipulator {

// Declaration order is frozen: a designated initializer must name members in declaration order, so
// reordering a slot breaks every project that already composes this aggregate. Appending is safe.
// The solves are here, and each receives what its own mathematics consumes and nothing else: the
// chain it is to solve over, the pose it is to reach, and the maps it reads along the way. Nothing
// any of them is handed carries a way to ask for a solve.
struct inverse_kinematics_ops
{
    expected<void, refusal> (*inverse_kinematics)(const forward_kinematics_ops &forward, const differential_kinematics_ops &differential, const screw_chain &chain,
                                                  const transform &desired, const joint_vector &j0, const solver_parameters &parameters, ik_result &answer) = &inert::inverse_kinematics;
    expected<void, refusal> (*analytic_inverse_kinematics)(const forward_kinematics_ops &forward, const screw_chain &chain, const transform &desired,
                                                           ik_result &answer) = &inert::analytic_inverse_kinematics;
};

// Holds the chain the bound implementations are asked about and routes every question to them. It is
// not an interface: a capability is added by binding a slot, which leaves a project that binds none
// of them building unchanged.
class kinematics
{
public:
    kinematics();

    // Refuses a chain whose limits do not carry one entry per degree of freedom, which is the only
    // thing it needs of the chain itself. An unbound slot is not refused here: a capability refuses
    // where it is asked for, so binding one is never a precondition of binding another.
    static expected<kinematics, refusal> compose(screw_chain chain, forward_kinematics_ops forward, differential_kinematics_ops differential, inverse_kinematics_ops inverse,
                                                 const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames);

    std::uint32_t joint_count() const;

    const screw_chain &space_chain() const;

    // The body chain is derived once, at composition, from the slot that derives it. A derivation
    // that refused leaves no chain and no Jacobian to take in one, and the refusal it produced is
    // what both of these answer with. A chain with no screws needs no derivation, so its empty body
    // chain is an answer rather than a refusal.
    expected<std::reference_wrapper<const screw_chain>, refusal> body_chain() const;

    expected<transform, refusal> fk_solve(const joint_vector &joint_positions) const;

    // Asked over the derived body chain, so a derivation that refused is answered for here with the
    // refusal it produced, as it is at the body Jacobian.
    expected<transform, refusal> body_fk_solve(const joint_vector &joint_positions) const;

    expected<jacobian, refusal> space_jacobian(const joint_vector &joint_positions) const;

    expected<jacobian, refusal> body_jacobian(const joint_vector &joint_positions) const;

    expected<joint_vector, refusal> ik_solve(const transform &desired_pose, const joint_vector &j0, const solver_parameters &parameters) const;
    expected<joint_vector, refusal> ik_solve(const transform &desired_pose, const joint_vector &j0, const solver_parameters &parameters,
                                             const std::function<std::optional<std::size_t>(std::span<const joint_vector>)> &solution_selector) const;

    // Every configuration that reaches the pose, answered in one go rather than searched for, so
    // neither a seed nor a stopping test enters. The span is the one solutions() reads and lasts
    // until the next solve on the same object.
    expected<std::span<const joint_vector>, refusal> configurations_reaching(const transform &desired_pose) const;

    // Every configuration the bound solver answered for the target of the last solve, in the order
    // it named them. The span stays valid until the next solve on the same object, and is empty
    // when the solver answered nothing. One strand owns a solver, so the span is read inside a
    // handler on that strand and what leaves the strand is a copy rather than the span.
    std::span<const joint_vector> solutions() const;

    // The span stays valid until the next solve on the same object, and is empty when the bound
    // solver reports no iterations. One strand owns a solver, so the span is read inside a handler
    // on that strand and what leaves the strand is a copy rather than the span.
    std::span<const iteration_state> iterations() const;

    // Entries into the solve rather than solutions found: a solve that ran and converged on nothing
    // raises this, and an operation that refused before reaching the solver does not.
    std::uint64_t solve_count() const;

private:
    kinematics(screw_chain chain, forward_kinematics_ops forward, differential_kinematics_ops differential, inverse_kinematics_ops inverse, const rigid_motion::frame_ops &frames);

    screw_chain m_space;
    screw_chain m_body;
    std::optional<refusal> m_body_unavailable;
    forward_kinematics_ops m_fk;
    differential_kinematics_ops m_dk;
    inverse_kinematics_ops m_ik;
    rigid_motion::frame_ops m_frames;
    mutable ik_result m_last;
    mutable std::uint64_t m_solve_tally;

    expected<void, refusal> solve(const transform &desired_pose, const joint_vector &j0, const solver_parameters &parameters) const;
};

}

#endif
