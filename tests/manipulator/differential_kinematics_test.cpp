#include "two_joint_bindings.h"

#include "praxis/manipulator/kinematics.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

using namespace praxis;
using namespace praxis::manipulator;
using namespace praxis::fixture;

TEST_CASE("a bound jacobian is what the solver answers a jacobian with")
{
    const kinematics solver  = holding({}, differential_kinematics_ops{.space_jacobian = &counting_space_jacobian}, {});
    const joint_vector theta = joint_vector::Constant(2, 0.5);

    const expected<jacobian, refusal> space_frame = solver.space_jacobian(theta);
    REQUIRE(space_frame.has_value());
    REQUIRE(space_frame->cols() == 2);
    CHECK(is_approx_equal((*space_frame)(0, 0), 2.0));
    REQUIRE_FALSE(solver.body_jacobian(theta).has_value());
}
