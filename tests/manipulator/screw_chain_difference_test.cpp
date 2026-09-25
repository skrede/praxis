#include "supplied_chain.h"

#include "praxis/manipulator/screw_chain.h"
#include "praxis/manipulator/screw_chain_difference.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;

TEST_CASE("a chain compared against itself reads zero on every term every line carries", "[manipulator][modeling]")
{
    const screw_chain derived          = a_chain();
    const screw_chain_difference apart = supplied_chain_difference(derived, derived.home, all_supplied(derived.space_screws));

    REQUIRE(apart.joints.size() == chain_joints);
    REQUIRE(apart.supplied == chain_joints);

    for(std::size_t joint = 0u; joint < chain_joints; ++joint)
    {
        const chain_joint_difference &line = apart.joints[joint];

        REQUIRE(line.read == chain_joint_reading::measured);
        REQUIRE(line.direction_radians.has_value());
        REQUIRE(line.length.has_value());
        REQUIRE(line.moment_metres.has_value() == (joint != translating_joint));
        CHECK(*line.direction_radians < exactly);
        CHECK(*line.length < exactly);
    }

    CHECK(*apart.joints[turning_joint].moment_metres < exactly);
    CHECK(apart.home.turned_radians < exactly);
    CHECK(apart.home.moved_metres < exactly);
    CHECK(apart.home.rigidity < exactly);
}

// A positive angle about a flipped direction turns the arm the other way, so the pair names a
// different chain rather than the same one up to a sign.
TEST_CASE("a direction flipped against the one described reads a half turn rather than agreement", "[manipulator][modeling]")
{
    const screw_chain derived              = a_chain();
    const std::vector<screw_axis> supplied = with_one_changed(derived.space_screws, turning_joint, -derived.space_screws[turning_joint]);
    const screw_chain_difference apart     = supplied_chain_difference(derived, derived.home, all_supplied(supplied));
    const chain_joint_difference &flipped  = apart.joints[turning_joint];

    REQUIRE(flipped.read == chain_joint_reading::measured);
    REQUIRE(flipped.direction_radians.has_value());
    REQUIRE(flipped.length.has_value());
    REQUIRE(flipped.moment_metres.has_value());
    CHECK(std::fabs(*flipped.direction_radians - a_half_turn) < nearly);
    CHECK(*flipped.length < exactly);
    CHECK(*flipped.moment_metres > nearly);

    for(std::size_t joint = 0u; joint < chain_joints; ++joint)
        if(joint != turning_joint)
        {
            CHECK(*apart.joints[joint].direction_radians < exactly);
            CHECK(*apart.joints[joint].length < exactly);
        }
}

TEST_CASE("a doubled axis whose line is exactly right reads its length and nothing else", "[manipulator][modeling]")
{
    const screw_chain derived              = a_chain();
    const std::vector<screw_axis> supplied = with_one_changed(derived.space_screws, turning_joint, 2.0 * derived.space_screws[turning_joint]);
    const screw_chain_difference apart     = supplied_chain_difference(derived, derived.home, all_supplied(supplied));
    const chain_joint_difference &doubled  = apart.joints[turning_joint];

    REQUIRE(doubled.read == chain_joint_reading::measured);
    REQUIRE(doubled.direction_radians.has_value());
    REQUIRE(doubled.length.has_value());
    REQUIRE(doubled.moment_metres.has_value());
    CHECK(*doubled.direction_radians < exactly);
    CHECK(std::fabs(*doubled.length - 1.0) < exactly);
    CHECK(*doubled.moment_metres < exactly);
}

// A translation is the same motion wherever its axis is put, so there is no moment to compare.
TEST_CASE("two screws that both translate read the angle between their linear halves and no moment", "[manipulator][modeling]")
{
    const screw_chain derived              = a_chain();
    const std::vector<screw_axis> supplied = with_one_changed(derived.space_screws, translating_joint, axis_of(0.0, 0.0, 0.0, 1.5, 0.0, 0.0));
    const screw_chain_difference apart     = supplied_chain_difference(derived, derived.home, all_supplied(supplied));
    const chain_joint_difference &shifted  = apart.joints[translating_joint];

    REQUIRE(shifted.read == chain_joint_reading::measured);
    REQUIRE(shifted.direction_radians.has_value());
    REQUIRE(shifted.length.has_value());
    REQUIRE(!shifted.moment_metres.has_value());
    CHECK(std::fabs(*shifted.direction_radians - a_quarter_turn) < nearly);
    CHECK(std::fabs(*shifted.length - 0.5) < exactly);
}

TEST_CASE("one screw turning against one translating fills no term at all and says the kinds differ", "[manipulator][modeling]")
{
    const screw_chain derived              = a_chain();
    const std::vector<screw_axis> supplied = with_one_changed(derived.space_screws, translating_joint, axis_of(0.0, 1.0, 0.0, 0.0, 0.0, 0.0));
    const screw_chain_difference apart     = supplied_chain_difference(derived, derived.home, all_supplied(supplied));
    const chain_joint_difference &mixed    = apart.joints[translating_joint];

    REQUIRE(mixed.read == chain_joint_reading::kinds_differed);
    CHECK(!mixed.direction_radians.has_value());
    CHECK(!mixed.length.has_value());
    CHECK(!mixed.moment_metres.has_value());
}

TEST_CASE("a translating screw whose linear half has no length names no direction", "[manipulator][modeling]")
{
    const screw_chain derived              = a_chain();
    const std::vector<screw_axis> supplied = with_one_changed(derived.space_screws, translating_joint, axis_of(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    const screw_chain_difference apart     = supplied_chain_difference(derived, derived.home, all_supplied(supplied));
    const chain_joint_difference &nowhere  = apart.joints[translating_joint];

    REQUIRE(nowhere.read == chain_joint_reading::measured);
    CHECK(!nowhere.direction_radians.has_value());
    CHECK(!nowhere.moment_metres.has_value());
    REQUIRE(nowhere.length.has_value());
    CHECK(std::fabs(*nowhere.length - 1.0) < exactly);
}

TEST_CASE("the length term is symmetric in which of the two chains is called the derived one", "[manipulator][modeling]")
{
    const screw_chain derived              = a_chain();
    const std::vector<screw_axis> supplied = with_one_changed(derived.space_screws, turning_joint, 2.0 * derived.space_screws[turning_joint]);
    const screw_chain swapped(derived.home, supplied, derived.limits);

    const screw_chain_difference one_way   = supplied_chain_difference(derived, derived.home, all_supplied(supplied));
    const screw_chain_difference other_way = supplied_chain_difference(swapped, derived.home, all_supplied(derived.space_screws));

    REQUIRE(one_way.joints.size() == other_way.joints.size());

    for(std::size_t joint = 0u; joint < one_way.joints.size(); ++joint)
    {
        REQUIRE(one_way.joints[joint].length.has_value() == other_way.joints[joint].length.has_value());
        CHECK(std::fabs(*one_way.joints[joint].length - *other_way.joints[joint].length) < exactly);
    }
}
