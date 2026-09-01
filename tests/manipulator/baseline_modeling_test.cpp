#include "recorded_chain.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/modeling.h"
#include "praxis/manipulator/baseline/kinematics.h"

#include "praxis/evaluation/tolerance.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <meios/model.h>

#include <cstddef>

using namespace praxis;
using namespace praxis::manipulator;
using namespace praxis::fixture;

namespace {

// The shape the builder's own fixture carries, reduced to the single joint this case is about: an
// actuated joint whose axis vector is zero, which no rotation into the root frame can turn into a
// direction.
meios::model<> zero_axis_arm()
{
    meios::link<> base{};
    base.name = "base";
    meios::link<> shoulder{};
    shoulder.name = "shoulder";

    meios::joint<> revolute{};
    revolute.name   = "shoulder";
    revolute.kind   = meios::joint_kind::revolute;
    revolute.parent = "base";
    revolute.child  = "shoulder";
    revolute.axis   = {0.0, 0.0, 0.0};

    meios::model<> model;
    model.name   = "zero_axis_arm";
    model.links  = {base, shoulder};
    model.joints = {revolute};

    meios::log_sink silent;
    model.topo = meios::reconstruct_topology(model.links, model.joints, silent, meios::topology_policy::skip).topo;

    return model;
}

}

TEST_CASE("the_derived_chain_carries_the_joint_count_home_pose_and_screw_axes_recorded_for_the_deployed_robot")
{
    const auto description = deployed_description();
    if(!description)
        SKIP("no robot description is deployed for this configuration");

    const auto derived = build_chain(*description);
    REQUIRE(derived.has_value());

    const screw_chain &chain             = *derived;
    const std::vector<screw_axis> screws = recorded_screws();

    REQUIRE(chain.joint_count() == recorded_joint_count);
    REQUIRE(chain.space_screws.size() == recorded_joint_count);
    CHECK(is_approx_equal(chain.home, recorded_home(), 1.0e-12));
    for(std::size_t i = 0; i < recorded_joint_count; ++i)
        CHECK((chain.space_screws[i] - screws[i]).isZero(1.0e-12));
}

TEST_CASE("the_derived_joint_bounds_are_the_ones_the_description_declares")
{
    const auto description = deployed_description();
    if(!description)
        SKIP("no robot description is deployed for this configuration");

    const auto derived = build_chain(*description);
    REQUIRE(derived.has_value());

    const screw_chain &chain = *derived;

    CHECK(is_approx_equal(chain.limits.lower_position, recorded_lower_bounds()));
    CHECK(is_approx_equal(chain.limits.upper_position, recorded_upper_bounds()));
    CHECK(is_approx_equal(chain.limits.velocity, recorded_velocity_bounds()));
    CHECK(is_approx_equal(chain.limits.acceleration, joint_vector(recorded_acceleration_ratio * recorded_velocity_bounds())));
}

TEST_CASE("forward_kinematics_through_the_derived_chain_reproduces_the_recorded_home_and_raised_poses")
{
    const auto description = deployed_description();
    if(!description)
        SKIP("no robot description is deployed for this configuration");

    const auto derived = build_chain(*description);
    REQUIRE(derived.has_value());

    const screw_chain &chain = *derived;
    const kinematics solver  = make_kinematics(chain, baseline().fk, baseline().dk, baseline().ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame).value();

    REQUIRE(solver.joint_count() == recorded_joint_count);
    CHECK(is_approx_equal(solver.fk_solve(joint_vector::Zero(recorded_joint_count)).value(), recorded_home(), 1.0e-12));
    CHECK(is_approx_equal(solver.fk_solve(recorded_raised_configuration()).value(), recorded_raised_pose(), 1.0e-12));
}

TEST_CASE("the_derived_home_pose_is_a_rigid_motion_and_every_derived_screw_axis_is_normalized")
{
    const auto description = deployed_description();
    if(!description)
        SKIP("no robot description is deployed for this configuration");

    const auto derived = build_chain(*description);
    REQUIRE(derived.has_value());

    const screw_chain &chain = *derived;

    CHECK(is_rigid_motion(chain.home));
    for(const screw_axis &axis : chain.space_screws)
        CHECK(is_unit_screw(axis));
}

TEST_CASE("a_description_whose_only_actuated_joint_has_a_zero_axis_is_refused_as_degenerate")
{
    const auto derived = build_chain(zero_axis_arm());

    REQUIRE_FALSE(derived.has_value());
    REQUIRE(derived.error() == refusal::degenerate);
}
