#include "two_joint_bindings.h"

#include "praxis/manipulator/kinematics.h"

#include "praxis/evaluation/tolerance.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <functional>

using namespace praxis;
using namespace praxis::manipulator;
using namespace praxis::fixture;

// Forward kinematics is one assignment and the Jacobians and the solve are others, so binding that
// one slot has to be enough on its own: composing succeeds over a chain with screws, the forward map
// answers, and every other slot refuses rather than answering in its place.
TEST_CASE("a composition binding forward kinematics alone answers with it and refuses elsewhere")
{
    const expected<kinematics, refusal> composed = kinematics::compose(two_joint_chain(), forward_kinematics_ops{.forward_kinematics = &lifting_forward_kinematics}, {}, {},
                                                                       rigid_motion::baseline().screw, rigid_motion::baseline().frame);
    const joint_vector theta                     = joint_vector::Constant(2, 0.5);

    REQUIRE(composed.has_value());
    CHECK(composed->joint_count() == 2u);
    REQUIRE(composed->fk_solve(theta).has_value());
    CHECK(is_approx_equal((*composed->fk_solve(theta))(2, 3), 0.5));

    const expected<jacobian, refusal> space_frame = composed->space_jacobian(theta);
    const expected<jacobian, refusal> body_frame  = composed->body_jacobian(theta);
    const expected<joint_vector, refusal> solved  = composed->ik_solve(transform::Identity(), theta, solver_parameters());

    REQUIRE_FALSE(space_frame.has_value());
    REQUIRE_FALSE(body_frame.has_value());
    REQUIRE_FALSE(solved.has_value());
    CHECK(space_frame.error() == refusal::not_implemented);
    CHECK(body_frame.error() == refusal::not_implemented);
    CHECK(solved.error() == refusal::not_implemented);
}

// The derivation is a capability like any other: a composition that leaves it unbound is composed,
// and what is missing surfaces at both answers that need a body chain rather than at the
// composition, which needs none.
TEST_CASE("the body chain is derived through the slot that derives it")
{
    const kinematics unbound    = holding({}, {}, {});
    const kinematics overridden = holding(forward_kinematics_ops{.body_screws_from_space = &mirrored_body_screws}, {}, {});
    const joint_vector theta    = joint_vector::Constant(2, 0.5);

    const expected<std::reference_wrapper<const screw_chain>, refusal> unavailable_chain = unbound.body_chain();
    const expected<jacobian, refusal> unavailable_frame                                  = unbound.body_jacobian(theta);

    REQUIRE_FALSE(unavailable_chain.has_value());
    REQUIRE_FALSE(unavailable_frame.has_value());
    CHECK(unavailable_chain.error() == refusal::not_implemented);
    CHECK(unavailable_frame.error() == refusal::not_implemented);

    const expected<std::reference_wrapper<const screw_chain>, refusal> derived = overridden.body_chain();
    REQUIRE(derived.has_value());
    CHECK(derived->get().joint_count() == 2u);
    CHECK(is_approx_equal(derived->get().space_screws.front()[0], 3.0));
}

// The derived body chain has a second reader. What says the holder passed that chain and not the space
// chain is the screw the bound slot reads back: the derivation here answers threes and the space chain
// carries zeros.
TEST_CASE("the body form is asked over the derived body chain rather than over the space chain")
{
    const kinematics solver =
            holding(forward_kinematics_ops{.body_forward_kinematics = &screw_reading_body_forward_kinematics, .body_screws_from_space = &mirrored_body_screws}, {}, {});
    const joint_vector theta = joint_vector::Constant(2, 0.5);

    const expected<transform, refusal> reached = solver.body_fk_solve(theta);
    REQUIRE(reached.has_value());
    CHECK(is_approx_equal((*reached)(2, 3), 3.0));
}

// The slot bound here answers whatever it is handed, so the refusal that comes out is the derivation's
// and the slot was never reached. The space form over the same composition needs no body chain and
// answers, which is what keeps one capability from being the price of another.
TEST_CASE("a derivation that refused is carried to the body form rather than to a slot that would answer")
{
    const kinematics refused = holding(forward_kinematics_ops{.forward_kinematics      = &lifting_forward_kinematics,
                                                              .body_forward_kinematics = &screw_reading_body_forward_kinematics,
                                                              .body_screws_from_space  = &unreadable_body_screws},
                                       {}, {});
    const joint_vector theta = joint_vector::Constant(2, 0.5);

    const expected<transform, refusal> from_body = refused.body_fk_solve(theta);
    REQUIRE_FALSE(from_body.has_value());
    CHECK(from_body.error() == refusal::degenerate);

    const expected<transform, refusal> from_space = refused.fk_solve(theta);
    REQUIRE(from_space.has_value());
    CHECK(is_approx_equal((*from_space)(2, 3), 0.5));
}

// An empty body chain is an answer rather than a refusal, so the body form over one is asked. The
// derivation bound here refuses everything and its enumerator appears nowhere, which is what says it
// was never consulted.
TEST_CASE("a chain with no screws answers the body form rather than refusing it")
{
    const expected<kinematics, refusal> composed = kinematics::compose(
            screw_chain(), forward_kinematics_ops{.body_forward_kinematics = &screw_reading_body_forward_kinematics, .body_screws_from_space = &unreadable_body_screws}, {}, {},
            rigid_motion::baseline().screw, rigid_motion::baseline().frame);

    REQUIRE(composed.has_value());

    const expected<transform, refusal> reached = composed->body_fk_solve(joint_vector());
    REQUIRE(reached.has_value());
    CHECK(is_approx_equal((*reached)(2, 3), 0.0));
}

// Four enumerators are in play and the derivation's is a fourth one, so an accessor that classified
// the absence itself would report the wrong one rather than merely reporting.
TEST_CASE("a derivation that refused is answered for with its own refusal, not one invented for it")
{
    const kinematics refused = holding(forward_kinematics_ops{.body_screws_from_space = &unreadable_body_screws}, {}, {});
    const kinematics unbound = holding({}, {}, {});
    const joint_vector theta = joint_vector::Constant(2, 0.5);

    const expected<jacobian, refusal> from_refusal                                        = refused.body_jacobian(theta);
    const expected<jacobian, refusal> from_unbound                                        = unbound.body_jacobian(theta);
    const expected<std::reference_wrapper<const screw_chain>, refusal> chain_from_refusal = refused.body_chain();

    REQUIRE_FALSE(from_refusal.has_value());
    REQUIRE_FALSE(from_unbound.has_value());
    REQUIRE_FALSE(chain_from_refusal.has_value());
    CHECK(from_refusal.error() == refusal::degenerate);
    CHECK(from_unbound.error() == refusal::not_implemented);
    CHECK(chain_from_refusal.error() == refusal::degenerate);
}

// An empty span of space screws has no adjoint to apply, which is why composing over one asks for no
// derivation. A derivation that refuses everything is bound here and its enumerator appears nowhere
// afterwards, which is what says it was never consulted.
TEST_CASE("a chain with no screws is composed without asking for a derivation")
{
    const expected<kinematics, refusal> composed = kinematics::compose(screw_chain(), forward_kinematics_ops{.body_screws_from_space = &unreadable_body_screws}, {}, {},
                                                                       rigid_motion::baseline().screw, rigid_motion::baseline().frame);

    REQUIRE(composed.has_value());
    CHECK(composed->joint_count() == 0u);

    const expected<std::reference_wrapper<const screw_chain>, refusal> body = composed->body_chain();
    REQUIRE(body.has_value());
    CHECK(body->get().joint_count() == 0u);

    const expected<jacobian, refusal> body_frame = composed->body_jacobian(joint_vector());
    REQUIRE_FALSE(body_frame.has_value());
    CHECK(body_frame.error() == refusal::not_implemented);
}
