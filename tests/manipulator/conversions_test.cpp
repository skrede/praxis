#include "praxis/manipulator/conversions.h"

#include "praxis/rigid_motion/types.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <vector>

using namespace praxis;
using namespace praxis::manipulator;

namespace {

joint_vector test_joint_vector()
{
    joint_vector joints(4);
    joints << 2.0, 1.0, 6.0, 9.0;
    return joints;
}

std::vector<double> test_std_vector()
{
    return {2.0, 1.0, 6.0, 9.0};
}

}

TEST_CASE("a_joint_vector_survives_a_round_trip_through_std_vector")
{
    REQUIRE(to_std_vector(test_joint_vector()) == test_std_vector());
    REQUIRE(is_approx_equal(to_joint_vector(to_std_vector(test_joint_vector())), test_joint_vector()));
}

TEST_CASE("a_std_vector_survives_a_round_trip_through_a_joint_vector")
{
    REQUIRE(is_approx_equal(to_joint_vector(test_std_vector()), test_joint_vector()));
    REQUIRE(to_std_vector(to_joint_vector(test_std_vector())) == test_std_vector());
}

TEST_CASE("an_empty_vector_converts_both_ways")
{
    REQUIRE(to_std_vector(joint_vector()).empty());
    REQUIRE(to_joint_vector({}).size() == 0);
}

TEST_CASE("vectors_of_differing_length_never_compare_equal")
{
    joint_vector shorter(3);
    shorter << 2.0, 1.0, 6.0;
    REQUIRE(!is_approx_equal(shorter, test_joint_vector()));
    REQUIRE(!is_approx_equal(test_joint_vector(), shorter));
}

TEST_CASE("a_difference_under_the_default_tolerance_compares_equal")
{
    joint_vector nudged = test_joint_vector();
    nudged[2] += default_tolerance / 2.0;

    REQUIRE(is_approx_equal(nudged, test_joint_vector()));
    REQUIRE(is_approx_equal(nudged[2], test_joint_vector()[2]));
}

TEST_CASE("a_difference_over_the_default_tolerance_compares_unequal")
{
    joint_vector nudged = test_joint_vector();
    nudged[2] += default_tolerance * 2.0;

    REQUIRE(!is_approx_equal(nudged, test_joint_vector()));
    REQUIRE(!is_approx_equal(nudged[2], test_joint_vector()[2]));
}

TEST_CASE("poses_compare_within_the_tolerance_they_are_given")
{
    transform first  = transform::Identity();
    transform second = transform::Identity();
    second(0, 3)     = default_tolerance * 2.0;

    REQUIRE(is_approx_equal(first, first));
    REQUIRE(!is_approx_equal(first, second));
    REQUIRE(is_approx_equal(first, second, default_tolerance * 10.0));
}
