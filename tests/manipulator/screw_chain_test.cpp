#include "recorded_chain.h"

#include "praxis/manipulator/screw_chain.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/kinematics.h"
#include "praxis/manipulator/screw_chain_builder.h"

#include "praxis/evaluation/tolerance.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <meios/model.h>

#include <cstddef>
#include <algorithm>

using namespace praxis;
using namespace praxis::fixture;

namespace {

std::size_t actuated_joint_count(const meios::model<> &model)
{
    return static_cast<std::size_t>(std::count_if(model.joints.begin(), model.joints.end(), [](const meios::joint<> &record) { return record.kind != meios::joint_kind::fixed; }));
}

}

TEST_CASE("a chain derived from a deployed description carries every actuated joint the description declares")
{
    const auto description = deployed_description();
    if(!description)
        SKIP("no robot description is deployed for this configuration");

    const auto derived = manipulator::build_screw_chain(*description);
    REQUIRE(derived.has_value());

    const manipulator::screw_chain &chain = *derived;

    REQUIRE(chain.joint_count() == actuated_joint_count(*description));
    REQUIRE(chain.space_screws.size() == chain.joint_count());
}

TEST_CASE("the derived home pose is a rigid motion and every derived screw axis is normalized")
{
    const auto description = deployed_description();
    if(!description)
        SKIP("no robot description is deployed for this configuration");

    const auto derived = manipulator::build_screw_chain(*description);
    REQUIRE(derived.has_value());

    const manipulator::screw_chain &chain = *derived;

    REQUIRE(is_rigid_motion(chain.home));
    for(const screw_axis &axis : chain.space_screws)
        REQUIRE(is_unit_screw(axis));
}

TEST_CASE("the derived joint limits carry one entry per joint in all four vectors")
{
    const auto description = deployed_description();
    if(!description)
        SKIP("no robot description is deployed for this configuration");

    const auto derived = manipulator::build_screw_chain(*description);
    REQUIRE(derived.has_value());

    const manipulator::screw_chain &chain = *derived;
    const Eigen::Index count              = static_cast<Eigen::Index>(chain.joint_count());

    REQUIRE(chain.limits.velocity.size() == count);
    REQUIRE(chain.limits.acceleration.size() == count);
    REQUIRE(chain.limits.lower_position.size() == count);
    REQUIRE(chain.limits.upper_position.size() == count);
    REQUIRE((chain.limits.lower_position.array() < chain.limits.upper_position.array()).all());
}

// The property a hand-derived model has to satisfy as well: the pose the chain records as its home
// is the pose the forward map produces at the configuration where no joint has moved.
TEST_CASE("forward kinematics at the zero configuration reproduces the chain's own home pose")
{
    const auto description = deployed_description();
    if(!description)
        SKIP("no robot description is deployed for this configuration");

    const auto derived = manipulator::build_screw_chain(*description);
    REQUIRE(derived.has_value());

    const manipulator::screw_chain &chain = *derived;
    const manipulator::kinematics solver  = manipulator::make_kinematics(chain, manipulator::baseline().fk, manipulator::baseline().dk, manipulator::baseline().ik,
                                                                         rigid_motion::baseline().screw, rigid_motion::baseline().frame)
                                                    .value();

    const Eigen::Index count = static_cast<Eigen::Index>(chain.joint_count());

    REQUIRE(static_cast<std::size_t>(solver.joint_count()) == chain.joint_count());
    REQUIRE(is_approx_equal(solver.fk_solve(manipulator::joint_vector::Zero(count)).value(), chain.home, default_tolerance));
}
