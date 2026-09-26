#include "supplied_chain.h"

#include "praxis/manipulator/screw_chain.h"
#include "praxis/manipulator/screw_chain_difference.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <span>
#include <cmath>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;

namespace {

// A block orthonormal only to about a hundred-thousandth, which is what a pose copied off the page to
// four decimal places is.
rotation rounded_to_four_decimals(const rotation &block)
{
    return ((block.array() * 1.0e4).round() / 1.0e4).matrix();
}

rotation with_one_column_negated(const rotation &block)
{
    rotation turned = block;
    turned.col(0)   = -block.col(0);

    return turned;
}

}

TEST_CASE("a supplied span of no screws leaves every joint unsupplied and the home line still answered", "[manipulator][modeling]")
{
    const screw_chain derived          = a_chain();
    const screw_chain_difference apart = supplied_chain_difference(derived, derived.home, std::span<const supplied_screw>());

    REQUIRE(apart.supplied == 0u);
    REQUIRE(apart.joints.size() == chain_joints);

    for(const chain_joint_difference &line : apart.joints)
    {
        CHECK(line.read == chain_joint_reading::not_supplied);
        CHECK(!line.direction_radians.has_value());
        CHECK(!line.length.has_value());
        CHECK(!line.moment_metres.has_value());
    }

    CHECK(std::isfinite(apart.home.turned_radians));
    CHECK(std::isfinite(apart.home.moved_metres));
    CHECK(std::isfinite(apart.home.rigidity));
}

TEST_CASE("a supplied span shorter than the chain reads what it reaches and leaves the rest unsupplied", "[manipulator][modeling]")
{
    const screw_chain derived = a_chain();
    const std::vector<screw_axis> supplied(derived.space_screws.begin(), derived.space_screws.begin() + 2);

    const screw_chain_difference apart = supplied_chain_difference(derived, derived.home, all_supplied(supplied));

    REQUIRE(apart.supplied == 2u);
    REQUIRE(apart.joints.size() == chain_joints);
    CHECK(apart.joints[0].read == chain_joint_reading::measured);
    CHECK(apart.joints[1].read == chain_joint_reading::measured);
    CHECK(apart.joints[2].read == chain_joint_reading::not_supplied);
    CHECK(apart.joints[3].read == chain_joint_reading::not_supplied);
}

// A surplus is a fact of the answer a caller reads off the count, not an entry that was dropped.
TEST_CASE("a supplied span longer than the chain leaves the joint list the chain's own length", "[manipulator][modeling]")
{
    const screw_chain derived = a_chain();

    std::vector<screw_axis> supplied = derived.space_screws;
    supplied.insert(supplied.end(), derived.space_screws.begin(), derived.space_screws.begin() + 2);

    const screw_chain_difference apart = supplied_chain_difference(derived, derived.home, all_supplied(supplied));

    REQUIRE(supplied.size() > derived.joint_count());
    CHECK(apart.joints.size() == derived.joint_count());
    CHECK(apart.supplied == supplied.size());
    CHECK(apart.supplied > apart.joints.size());
}

TEST_CASE("a derived chain of no joints answers the home line and an empty joint list", "[manipulator][modeling]")
{
    const screw_chain derived(a_home_pose(), std::vector<screw_axis>(), joint_limits{});
    const std::vector<screw_axis> supplied = described_screws();

    const screw_chain_difference apart = supplied_chain_difference(derived, derived.home, all_supplied(supplied));

    CHECK(apart.joints.empty());
    CHECK(apart.supplied == chain_joints);
    CHECK(std::isfinite(apart.home.turned_radians));
    CHECK(std::isfinite(apart.home.moved_metres));
    CHECK(std::isfinite(apart.home.rigidity));
}

TEST_CASE("a home pose typed to four decimals reads a finite turn beside a rigidity defect of its own", "[manipulator][modeling]")
{
    const screw_chain derived = a_chain();

    transform typed         = derived.home;
    typed.block<3, 3>(0, 0) = rounded_to_four_decimals(rotation(derived.home.block<3, 3>(0, 0)));

    const screw_chain_difference apart = supplied_chain_difference(derived, typed, all_supplied(derived.space_screws));

    REQUIRE(std::isfinite(apart.home.turned_radians));
    REQUIRE(std::isfinite(apart.home.moved_metres));
    REQUIRE(std::isfinite(apart.home.rigidity));
    CHECK(apart.home.turned_radians < 1.0e-3);
    CHECK(apart.home.moved_metres < exactly);
    CHECK(apart.home.rigidity > 1.0e-5);
}

// The nearest rotation to a block naming no frame is the decomposition's own choice, so the turn read
// against it measures nothing that was supplied -- least of all when it lands on the derived chain's
// own angle.
TEST_CASE("a home pose whose rotation block names no frame at all is refused a turn", "[manipulator][modeling]")
{
    const screw_chain derived          = a_chain();
    const screw_chain_difference apart = supplied_chain_difference(derived, home_pose_whose_rotation_is(no_frame_at_all()), all_supplied(derived.space_screws));

    REQUIRE(std::isfinite(apart.home.moved_metres));
    REQUIRE(std::isfinite(apart.home.rigidity));
    REQUIRE(apart.home.rigidity > 0.5);
    CHECK_FALSE(std::isfinite(apart.home.turned_radians));
    CHECK(std::fabs(apart.home.turned_radians - a_home_turn) > nearly);
}

TEST_CASE("a home pose whose rotation block keeps one direction of three is refused a turn", "[manipulator][modeling]")
{
    const screw_chain derived          = a_chain();
    const screw_chain_difference apart = supplied_chain_difference(derived, home_pose_whose_rotation_is(one_direction_only()), all_supplied(derived.space_screws));

    REQUIRE(std::isfinite(apart.home.moved_metres));
    REQUIRE(std::isfinite(apart.home.rigidity));
    REQUIRE(apart.home.rigidity > 0.5);
    CHECK_FALSE(std::isfinite(apart.home.turned_radians));
}

// The turn is read against the nearest rotation to the supplied block only while the block stands
// near enough to one for that rotation to be a fact about it. A reflection is exactly orthonormal
// with the determinant of the wrong sign, so which direction gets flipped to reach a rotation is the
// decomposition's choice and nothing the block says.
TEST_CASE("a home pose whose rotation block is a reflection is refused a turn beside a rigidity of order one", "[manipulator][modeling]")
{
    const screw_chain derived = a_chain();

    transform mirrored         = derived.home;
    mirrored.block<3, 3>(0, 0) = with_one_column_negated(rotation(derived.home.block<3, 3>(0, 0)));

    const screw_chain_difference apart = supplied_chain_difference(derived, mirrored, all_supplied(derived.space_screws));

    REQUIRE(std::isfinite(apart.home.moved_metres));
    REQUIRE(std::isfinite(apart.home.rigidity));
    CHECK_FALSE(std::isfinite(apart.home.turned_radians));
    CHECK(apart.home.rigidity > 1.0);
    CHECK(apart.home.rigidity < 10.0);
}
