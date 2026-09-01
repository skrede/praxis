#include "recorded_chain.h"
#include "baseline/opw_geometry.h"

#include "praxis/manipulator/baseline/kinematics.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <vector>
#include <cstddef>
#include <numbers>

using namespace praxis;
using namespace praxis::manipulator;
using namespace praxis::fixture;

namespace {

// The parameter set the solver library's own fixture carries for this arm, which a test may name
// even though the library may not.
constexpr double pinned_a1 = 0.025;
constexpr double pinned_a2 = -0.035;
constexpr double pinned_b  = 0.0;
constexpr double pinned_c1 = 0.400;
constexpr double pinned_c2 = 0.455;
constexpr double pinned_c3 = 0.420;
constexpr double pinned_c4 = 0.080;

constexpr double derived_tolerance = 1.0e-9;

forward_kinematics_ops reference_forward()
{
    return forward_kinematics_ops{.forward_kinematics = &forward_kinematics, .body_forward_kinematics = &body_forward_kinematics, .body_screws_from_space = &body_screws_from_space};
}

screw_chain with_screws(std::vector<screw_axis> screws)
{
    screw_chain chain  = recorded_arm();
    chain.space_screws = std::move(screws);

    return chain;
}

// A screw whose axis runs along the given direction through the given point, which is the one form
// every synthetic chain below is bent by.
screw_axis about(const Eigen::Vector3d &direction, const Eigen::Vector3d &through)
{
    screw_axis axis = screw_axis::Zero();
    axis.head<3>()  = direction.normalized();
    axis.tail<3>()  = through.cross(direction.normalized());

    return axis;
}

}

TEST_CASE("the deployed six-axis arm decomposes into the ortho-parallel lengths its geometry carries")
{
    const expected<cartan::opw_parameters<double>, refusal> derived = to_opw_parameters(recorded_arm());

    REQUIRE(derived.has_value());
    CHECK(is_approx_equal(derived->a1, pinned_a1, derived_tolerance));
    CHECK(is_approx_equal(derived->a2, pinned_a2, derived_tolerance));
    CHECK(is_approx_equal(derived->b, pinned_b, derived_tolerance));
    CHECK(is_approx_equal(derived->c1, pinned_c1, derived_tolerance));
    CHECK(is_approx_equal(derived->c2, pinned_c2, derived_tolerance));
    CHECK(is_approx_equal(derived->c3, pinned_c3, derived_tolerance));
    CHECK(is_approx_equal(derived->c4, pinned_c4, derived_tolerance));
}

TEST_CASE("the offsets and the sign corrections carry the convention the deployed arm is described in")
{
    const expected<cartan::opw_parameters<double>, refusal> derived = to_opw_parameters(recorded_arm());
    const std::array<signed char, 6> pinned_signs{-1, 1, 1, -1, 1, -1};

    REQUIRE(derived.has_value());
    CHECK(is_approx_equal(derived->offsets[0], 0.0, derived_tolerance));
    CHECK(is_approx_equal(derived->offsets[1], -std::numbers::pi / 2.0, derived_tolerance));
    for(std::size_t joint = 2; joint < 6; ++joint)
        CHECK(is_approx_equal(derived->offsets[joint], 0.0, derived_tolerance));
    CHECK(derived->sign_corrections == pinned_signs);
}

TEST_CASE("a chain of other than six joints is a size this decomposition does not serve")
{
    std::vector<screw_axis> five = recorded_screws();
    five.pop_back();

    const expected<cartan::opw_parameters<double>, refusal> derived = to_opw_parameters(with_screws(std::move(five)));

    REQUIRE_FALSE(derived.has_value());
    CHECK(derived.error() == refusal::unsupported_input);
}

TEST_CASE("a shoulder axis that is not orthogonal to the base axis is a kind this decomposition does not serve")
{
    std::vector<screw_axis> tilted = recorded_screws();
    tilted[1]                      = about(Eigen::Vector3d(0.0, 1.0, 1.0), Eigen::Vector3d(0.025, 0.0, 0.400));

    const expected<cartan::opw_parameters<double>, refusal> derived = to_opw_parameters(with_screws(std::move(tilted)));

    REQUIRE_FALSE(derived.has_value());
    CHECK(derived.error() == refusal::unsupported_input);
}

TEST_CASE("an elbow axis that is not parallel to the shoulder axis is a kind this decomposition does not serve")
{
    std::vector<screw_axis> skewed = recorded_screws();
    skewed[2]                      = about(Eigen::Vector3d(0.0, 1.0, 0.3), Eigen::Vector3d(0.480, 0.0, 0.400));

    const expected<cartan::opw_parameters<double>, refusal> derived = to_opw_parameters(with_screws(std::move(skewed)));

    REQUIRE_FALSE(derived.has_value());
    CHECK(derived.error() == refusal::unsupported_input);
}

// The wrist offsets of the smaller machine the demonstration also ships, which is the arm a learner
// meets that this closed form has no answer for.
TEST_CASE("last three axes that meet at no common point are a kind this decomposition does not serve")
{
    std::vector<screw_axis> apart = recorded_screws();
    apart[4]                      = about(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(0.900, 0.0, 0.520));

    const expected<cartan::opw_parameters<double>, refusal> derived = to_opw_parameters(with_screws(std::move(apart)));

    REQUIRE_FALSE(derived.has_value());
    CHECK(derived.error() == refusal::unsupported_input);
}

TEST_CASE("the derived parameters reconstruct the arm the chain describes")
{
    const screw_chain arm                                           = recorded_arm();
    const expected<cartan::opw_parameters<double>, refusal> derived = to_opw_parameters(arm);

    REQUIRE(derived.has_value());
    CHECK(agrees_with_chain(reference_forward(), arm, *derived).has_value());
}

// A quarter turn of the home pose is the failure a frame convention read the wrong way round would
// produce, and a tenth of a millimetre on one length is the smallest of the three the gate is meant
// to see; each moves the reconstruction rather than cancelling inside it.
TEST_CASE("parameters describing another arm than the chain does are refused by the reconstruction")
{
    screw_chain turned = recorded_arm();
    turned.home        = recorded_home() * transform(Eigen::Affine3d(Eigen::AngleAxisd(std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ())).matrix());

    const expected<cartan::opw_parameters<double>, refusal> derived = to_opw_parameters(recorded_arm());
    REQUIRE(derived.has_value());

    cartan::opw_parameters<double> shortened = *derived;
    shortened.c2 -= 1.0e-4;
    cartan::opw_parameters<double> mirrored = *derived;
    mirrored.sign_corrections[3]            = 1;

    const expected<void, refusal> against_a_turn = agrees_with_chain(reference_forward(), turned, *derived);
    REQUIRE_FALSE(against_a_turn.has_value());
    CHECK(against_a_turn.error() == refusal::unsupported_input);
    CHECK_FALSE(agrees_with_chain(reference_forward(), recorded_arm(), shortened).has_value());
    CHECK_FALSE(agrees_with_chain(reference_forward(), recorded_arm(), mirrored).has_value());
}
