#include "three_link_arm.h"
#include "substituted_rigid_motion.h"

#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/screw_chain.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <vector>
#include <cstddef>
#include <algorithm>
#include <functional>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

// How far apart in any one coefficient two derivations of the same chain must stand before this file
// counts them as different answers, far above the arithmetic's own noise.
constexpr double axes_apart_by = 1.0e-3;

rigid_motion::capabilities only_the_adjoint()
{
    rigid_motion::capabilities spatial                  = rigid_motion::baseline();
    spatial.screw.adjoint_matrix_from_rotation_position = &scaled_adjoint;

    return spatial;
}

rigid_motion::capabilities only_the_rotation()
{
    rigid_motion::capabilities spatial           = rigid_motion::baseline();
    spatial.frame.rotation_matrix_from_transform = &scaled_rotation;

    return spatial;
}

double axes_apart(std::span<const screw_axis> held, std::span<const screw_axis> against)
{
    double most = 0.0;
    for(std::size_t i = 0; i < held.size(); ++i)
        most = std::max(most, (against[i] - held[i]).cwiseAbs().maxCoeff());

    return most;
}

expected<std::vector<screw_axis>, refusal> body_screws_under(const rigid_motion::capabilities &spatial, const screw_chain &arm)
{
    return manipulator::baseline().fk.body_screws_from_space(spatial.screw, spatial.frame, arm.home, arm.space_screws);
}

// The derivation is the rigid-motion baseline's on either side, so what the body form is asked under
// is the substitution alone rather than a chain the substitution already refused to derive.
expected<transform, refusal> body_reached_under(const rigid_motion::capabilities &spatial, const screw_chain &arm, const joint_vector &at)
{
    const expected<std::vector<screw_axis>, refusal> body = body_screws_under(rigid_motion::baseline(), arm);
    if(!body)
        return praxis::unexpected(body.error());

    return manipulator::baseline().fk.body_forward_kinematics(spatial.frame, arm.home, *body, at);
}

// Answers whatever derivation the rigid-motion baseline gives, so a composition binding it stands and
// the body forward map is left as the only reader of the frame operations that composition named.
expected<std::vector<screw_axis>, refusal> body_screws_of_the_baseline(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &, const transform &m,
                                                                       std::span<const screw_axis> space_screws)
{
    return body_screws_under(rigid_motion::baseline(), screw_chain(m, std::vector<screw_axis>(space_screws.begin(), space_screws.end()), joint_limits{}));
}

}

TEST_CASE("the body-screw derivation carries the space screws through the adjoint the composition hands it", "[seam][routing]")
{
    const rigid_motion::capabilities reference = rigid_motion::baseline();
    const rigid_motion::capabilities perturbed = only_the_adjoint();
    const screw_chain arm                      = three_link_arm();
    const rotation spun                        = rigid_motion::rotate_z(0.3);
    const Eigen::Vector3d point{0.2, 0.3, 0.0};

    REQUIRE_FALSE(perturbed.screw.adjoint_matrix_from_rotation_position(spun, point)->isApprox(*reference.screw.adjoint_matrix_from_rotation_position(spun, point)));

    const expected<std::vector<screw_axis>, refusal> under_the_reference = body_screws_under(reference, arm);
    const expected<std::vector<screw_axis>, refusal> under_substitution  = body_screws_under(perturbed, arm);

    REQUIRE(under_the_reference.has_value());
    REQUIRE(under_substitution.has_value());
    REQUIRE(under_substitution->size() == under_the_reference->size());
    REQUIRE(axes_apart(*under_the_reference, *under_substitution) > axes_apart_by);
}

TEST_CASE("both body-frame answers read the home pose through the rotation extractor the composition hands them", "[seam][routing]")
{
    const rigid_motion::capabilities reference = rigid_motion::baseline();
    const rigid_motion::capabilities perturbed = only_the_rotation();
    const screw_chain arm                      = three_link_arm();
    const joint_vector at                      = posed_arm();

    REQUIRE_FALSE(perturbed.frame.rotation_matrix_from_transform(arm.home).isApprox(reference.frame.rotation_matrix_from_transform(arm.home)));
    REQUIRE(body_screws_under(reference, arm).has_value());
    REQUIRE(body_reached_under(reference, arm, at).has_value());

    const expected<std::vector<screw_axis>, refusal> derived = body_screws_under(perturbed, arm);
    const expected<transform, refusal> reached               = body_reached_under(perturbed, arm, at);

    REQUIRE_FALSE(derived.has_value());
    REQUIRE_FALSE(reached.has_value());
    CHECK(derived.error() == refusal::degenerate);
    CHECK(reached.error() == refusal::degenerate);
}

TEST_CASE("a solver composed against a rebound rotation extractor composes and carries that refusal to its body chain", "[seam][routing]")
{
    const capabilities arm                       = manipulator::baseline();
    const rigid_motion::capabilities perturbed   = only_the_rotation();
    const joint_vector at                        = posed_arm();
    const expected<kinematics, refusal> composed = kinematics::compose(three_link_arm(), arm.fk, arm.dk, arm.ik, perturbed.screw, perturbed.frame);

    REQUIRE(composed.has_value());
    REQUIRE(composed->fk_solve(at).has_value());
    REQUIRE(composed->space_jacobian(at).has_value());

    const expected<std::reference_wrapper<const screw_chain>, refusal> body = composed->body_chain();
    const expected<transform, refusal> reached                              = composed->body_fk_solve(at);

    REQUIRE_FALSE(body.has_value());
    REQUIRE_FALSE(reached.has_value());
    CHECK(body.error() == refusal::degenerate);
    CHECK(reached.error() == refusal::degenerate);
}

TEST_CASE("both body-frame answers refuse under a composition that substitutes every operation", "[seam][routing]")
{
    const rigid_motion::capabilities reference = rigid_motion::baseline();
    const rigid_motion::capabilities perturbed = substituted_everywhere();
    const screw_chain arm                      = three_link_arm();
    const joint_vector at                      = posed_arm();

    REQUIRE(every_substituted_slot_differs(reference, perturbed));
    REQUIRE(body_screws_under(reference, arm).has_value());
    REQUIRE(body_reached_under(reference, arm, at).has_value());

    const expected<std::vector<screw_axis>, refusal> derived = body_screws_under(perturbed, arm);
    const expected<transform, refusal> reached               = body_reached_under(perturbed, arm, at);

    REQUIRE_FALSE(derived.has_value());
    REQUIRE_FALSE(reached.has_value());
    CHECK(derived.error() == refusal::degenerate);
    CHECK(reached.error() == refusal::degenerate);
}

TEST_CASE("the body form a solver answers is taken under the frame operations its own composition named", "[seam][routing]")
{
    capabilities arm              = manipulator::baseline();
    arm.fk.body_screws_from_space = &body_screws_of_the_baseline;

    const rigid_motion::capabilities perturbed   = only_the_rotation();
    const joint_vector at                        = posed_arm();
    const expected<kinematics, refusal> composed = kinematics::compose(three_link_arm(), arm.fk, arm.dk, arm.ik, perturbed.screw, perturbed.frame);

    REQUIRE(composed.has_value());
    REQUIRE(composed->body_chain().has_value());

    const expected<transform, refusal> reached = composed->body_fk_solve(at);

    REQUIRE_FALSE(reached.has_value());
    CHECK(reached.error() == refusal::degenerate);
}
