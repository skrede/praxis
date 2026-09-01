#include "two_joint_bindings.h"

#include "praxis/manipulator/kinematics.h"

#include "praxis/evaluation/tolerance.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <vector>
#include <cstdint>
#include <functional>

using namespace praxis;
using namespace praxis::manipulator;
using namespace praxis::fixture;

TEST_CASE("every slot of a value-initialized composition refuses rather than answering")
{
    forward_kinematics_ops forward{};
    differential_kinematics_ops differential{};
    inverse_kinematics_ops inverse{};
    const joint_vector j0 = joint_vector::Constant(2, 0.25);
    ik_result answer;

    const expected<transform, refusal> reached               = forward.forward_kinematics(transform::Identity(), {}, j0);
    const expected<jacobian, refusal> space_frame            = differential.space_jacobian({}, j0);
    const expected<jacobian, refusal> body_frame             = differential.body_jacobian({}, j0);
    const expected<std::vector<screw_axis>, refusal> derived = forward.body_screws_from_space(rigid_motion::screw_ops{}, rigid_motion::frame_ops{}, transform::Identity(), {});
    const expected<void, refusal> solved                     = inverse.inverse_kinematics(forward, differential, screw_chain(), transform::Identity(), j0, solver_parameters(), answer);
    const expected<transform, refusal> body_reached          = forward.body_forward_kinematics(rigid_motion::frame_ops{}, transform::Identity(), {}, j0);

    REQUIRE_FALSE(reached.has_value());
    REQUIRE_FALSE(space_frame.has_value());
    REQUIRE_FALSE(body_frame.has_value());
    REQUIRE_FALSE(derived.has_value());
    REQUIRE_FALSE(solved.has_value());
    REQUIRE_FALSE(body_reached.has_value());
    CHECK(reached.error() == refusal::not_implemented);
    CHECK(space_frame.error() == refusal::not_implemented);
    CHECK(body_frame.error() == refusal::not_implemented);
    CHECK(derived.error() == refusal::not_implemented);
    CHECK(solved.error() == refusal::not_implemented);
    CHECK(body_reached.error() == refusal::not_implemented);
}

TEST_CASE("a composition binding nothing refuses every answer and still says what it holds")
{
    const kinematics solver{};
    const joint_vector j0 = joint_vector::Constant(3, 0.25);

    CHECK(solver.joint_count() == 0u);
    CHECK(solver.space_chain().joint_count() == 0u);
    CHECK(solver.iterations().empty());

    const expected<std::reference_wrapper<const screw_chain>, refusal> body = solver.body_chain();
    REQUIRE(body.has_value());
    CHECK(body->get().joint_count() == 0u);

    const expected<transform, refusal> reached      = solver.fk_solve(j0);
    const expected<transform, refusal> body_reached = solver.body_fk_solve(j0);
    const expected<jacobian, refusal> space_frame   = solver.space_jacobian(j0);
    const expected<jacobian, refusal> body_frame    = solver.body_jacobian(j0);
    const expected<joint_vector, refusal> solved    = solver.ik_solve(transform::Identity(), j0, solver_parameters());

    REQUIRE_FALSE(reached.has_value());
    REQUIRE_FALSE(body_reached.has_value());
    REQUIRE_FALSE(space_frame.has_value());
    REQUIRE_FALSE(body_frame.has_value());
    REQUIRE_FALSE(solved.has_value());
    CHECK(reached.error() == refusal::not_implemented);
    CHECK(body_reached.error() == refusal::not_implemented);
    CHECK(space_frame.error() == refusal::not_implemented);
    CHECK(body_frame.error() == refusal::not_implemented);
    CHECK(solved.error() == refusal::not_implemented);
}

TEST_CASE("each forwarder carries the refusal its slot produced rather than one of its own")
{
    const kinematics solver  = holding(forward_kinematics_ops{.forward_kinematics = &degenerate_forward_kinematics, .body_screws_from_space = &mirrored_body_screws},
                                       differential_kinematics_ops{.space_jacobian = &unsupported_space_jacobian, .body_jacobian = &exhausted_body_jacobian},
                                       inverse_kinematics_ops{.inverse_kinematics = &degenerate_inverse_kinematics});
    const joint_vector theta = joint_vector::Constant(2, 0.5);

    const expected<transform, refusal> reached    = solver.fk_solve(theta);
    const expected<jacobian, refusal> space_frame = solver.space_jacobian(theta);
    const expected<jacobian, refusal> body_frame  = solver.body_jacobian(theta);
    const expected<joint_vector, refusal> solved  = solver.ik_solve(transform::Identity(), theta, solver_parameters());

    REQUIRE_FALSE(reached.has_value());
    REQUIRE_FALSE(space_frame.has_value());
    REQUIRE_FALSE(body_frame.has_value());
    REQUIRE_FALSE(solved.has_value());
    CHECK(reached.error() == refusal::degenerate);
    CHECK(space_frame.error() == refusal::unsupported_input);
    CHECK(body_frame.error() == refusal::no_solution);
    CHECK(solved.error() == refusal::degenerate);
}

TEST_CASE("the holder answers with the chain it was handed whatever is bound")
{
    const screw_chain given = two_joint_chain();
    const kinematics solver = holding(forward_kinematics_ops{.body_screws_from_space = &mirrored_body_screws}, {}, {});

    REQUIRE(solver.joint_count() == static_cast<std::uint32_t>(given.joint_count()));
    CHECK(is_approx_equal(solver.space_chain().home, given.home));
    REQUIRE(solver.space_chain().space_screws.size() == given.space_screws.size());
    CHECK((solver.space_chain().space_screws.front() - given.space_screws.front()).isZero(default_tolerance));

    const expected<std::reference_wrapper<const screw_chain>, refusal> body = solver.body_chain();
    REQUIRE(body.has_value());
    CHECK(body->get().joint_count() == given.joint_count());
    CHECK(is_approx_equal(body->get().home, given.home));
}

// Every control that reads the limits indexes all four of them by joint with no bound of its own, so
// a chain whose limits do not carry one entry per degree of freedom is refused rather than held.
TEST_CASE("a chain whose limits do not carry one entry per joint is refused")
{
    const std::vector<screw_axis> screws = {screw_axis::Zero(), screw_axis::Zero()};

    joint_limits absent_velocity    = two_joint_bounds();
    absent_velocity.velocity        = joint_vector();
    joint_limits short_acceleration = two_joint_bounds();
    short_acceleration.acceleration = joint_vector::Constant(1, 4.0);
    joint_limits long_lower         = two_joint_bounds();
    long_lower.lower_position       = joint_vector::Constant(3, -1.5);
    joint_limits absent_upper       = two_joint_bounds();
    absent_upper.upper_position     = joint_vector();

    for(const joint_limits &bounds : {absent_velocity, short_acceleration, long_lower, absent_upper})
    {
        const expected<kinematics, refusal> composed =
                kinematics::compose(screw_chain(transform::Identity(), screws, bounds), {}, {}, {}, rigid_motion::baseline().screw, rigid_motion::baseline().frame);
        REQUIRE_FALSE(composed.has_value());
        CHECK(composed.error() == refusal::unsupported_input);
    }

    CHECK(kinematics::compose(two_joint_chain(), {}, {}, {}, rigid_motion::baseline().screw, rigid_motion::baseline().frame).has_value());
}
