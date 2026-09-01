#include "fixtures.h"
#include "drawn_chain.h"
#include "drawn_ellipsoids.h"

#include "../presets/drawn_lines.h"

#include "robot/ellipsoid_figure.h"

#include "praxis/manipulator/loadable_robot_stencil.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <threepp/math/Color.hpp>

#include <threepp/materials/interfaces.hpp>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <vector>
#include <cstddef>
#include <optional>

using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

// The condition number at or above which a body wears the ramp's last step.
constexpr double ramp_ceiling = 1.0e4;

threepp::Color tone_of(threepp::Object3D *drawn)
{
    REQUIRE(drawn != nullptr);
    const auto *shaded = drawn->materialAs<threepp::MaterialWithColor>();
    REQUIRE(shaded != nullptr);

    return shaded->color;
}

threepp::Color tone_at(std::size_t step)
{
    return std::dynamic_pointer_cast<threepp::MaterialWithColor>(ellipsoid_material(step, false))->color;
}

bool wireframed(threepp::Object3D *drawn)
{
    REQUIRE(drawn != nullptr);
    const auto *shaded = drawn->materialAs<threepp::MaterialWithWireframe>();

    return shaded != nullptr && shaded->wireframe;
}

}

TEST_CASE("telling the ellipsoids raises one body and six lines per block, under two subtrees neither of which is the other's", "[manipulator][drawing]")
{
    ellipsoid_stage headless;
    CHECK(headless.body(jacobian_block::angular) == nullptr);

    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::velocity);
    CHECK(wireframed(headless.body(jacobian_block::angular)));
    CHECK_FALSE(wireframed(headless.body(jacobian_block::linear)));

    headless.put(published(decomposed_both(Eigen::Vector3d(1.0, 0.6, 0.2)), refused(), Eigen::Vector3d(Eigen::Vector3d::Zero())));
    headless.draw();

    for(const jacobian_block which : {jacobian_block::angular, jacobian_block::linear})
    {
        CHECK(headless.body(which) != nullptr);
        for(std::size_t axis = 0; axis < 3u; ++axis)
            for(const bool forward : {true, false})
                CHECK(headless.line(which, axis, forward) != nullptr);
    }

    CHECK(headless.body(jacobian_block::angular)->parent != headless.body(jacobian_block::linear)->parent);
    CHECK(wireframed(headless.body(jacobian_block::angular)));
    CHECK_FALSE(wireframed(headless.body(jacobian_block::linear)));
}

TEST_CASE("every combination of the seven switches leaves the scene in the state those seven name", "[manipulator][drawing]")
{
    ellipsoid_stage headless;
    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::velocity);
    REQUIRE(headless.shown.set_solution_figures(std::vector<joint_vector>{configuration(0.2, 0.4)}).has_value());
    headless.put(published(decomposed_both(Eigen::Vector3d(1.0, 0.6, 0.2)), refused(), Eigen::Vector3d(Eigen::Vector3d::Zero())));

    for(int combination = 0; combination < 128; ++combination)
    {
        const bool meshes  = (combination & 1) != 0;
        const bool axes    = (combination & 2) != 0;
        const bool stick   = (combination & 4) != 0;
        const bool path    = (combination & 8) != 0;
        const bool figures = (combination & 16) != 0;
        const bool angular = (combination & 32) != 0;
        const bool linear  = (combination & 64) != 0;

        headless.shown.set_meshes_shown(meshes);
        headless.shown.set_decoration_shown(axes);
        headless.shown.set_chain_shown(stick);
        REQUIRE(headless.shown.set_pose_path_shown("recorded", path).has_value());
        headless.shown.set_solution_figures_shown(figures);
        headless.shown.set_angular_ellipsoid_shown(angular);
        headless.shown.set_linear_ellipsoid_shown(linear);
        headless.draw();

        CHECK(drawn(rendered_arm(*headless.scene)) == meshes);
        CHECK(drawn(first_line_under(*headless.scene, loadable_robot_stencil::joint_axis_name(0))) == axes);
        CHECK(drawn(chain_node(*headless.scene, loadable_robot_stencil::chain_name())) == stick);
        CHECK(drawn(first_line_under(*headless.scene, loadable_robot_stencil::pose_path_name("recorded"))) == path);
        CHECK(drawn(chain_node(*headless.scene, loadable_robot_stencil::solution_figure_name(0))) == figures);
        CHECK(drawn(headless.body(jacobian_block::angular)) == angular);
        CHECK(drawn(headless.body(jacobian_block::linear)) == linear);
    }
}

TEST_CASE("the tone a body wears is the ramp step its own condition number falls in", "[manipulator][drawing]")
{
    ellipsoid_stage headless;
    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::velocity);
    headless.put(published(jacobian_manipulability{decomposed(Eigen::Vector3d(1.0, 1.0, 1.0)), decomposed(Eigen::Vector3d(1.0, 0.5, 0.0))}, refused(),
                           Eigen::Vector3d(Eigen::Vector3d::Zero())));
    headless.draw();

    CHECK(ellipsoid_ramp_step(1.0) == 0u);
    CHECK(ellipsoid_ramp_step(std::nullopt) == ellipsoid_ramp_steps() - 1u);
    CHECK(tone_of(headless.body(jacobian_block::angular)).equals(tone_at(ellipsoid_ramp_step(1.0))));
    CHECK(tone_of(headless.body(jacobian_block::linear)).equals(tone_at(ellipsoid_ramp_step(std::nullopt))));
    CHECK_FALSE(tone_of(headless.body(jacobian_block::angular)).equals(tone_of(headless.body(jacobian_block::linear))));
}

TEST_CASE("clearing the ellipsoids takes both bodies out of the scene and leaves them to be told again", "[manipulator][drawing]")
{
    ellipsoid_stage headless;
    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::velocity);
    headless.put(published(decomposed_both(Eigen::Vector3d(1.0, 0.6, 0.2)), refused(), Eigen::Vector3d(Eigen::Vector3d::Zero())));
    headless.draw();
    REQUIRE(headless.body(jacobian_block::linear) != nullptr);

    headless.shown.clear_manipulability_ellipsoids();
    headless.draw();

    CHECK(headless.body(jacobian_block::angular) == nullptr);
    CHECK(headless.line(jacobian_block::linear, 0u, true) == nullptr);

    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::velocity);
    headless.draw();

    CHECK(drawn(headless.body(jacobian_block::linear)));
}

TEST_CASE("the ramp reaches its last step exactly at the ceiling condition number and its first at one", "[manipulator][drawing]")
{
    const std::size_t last = ellipsoid_ramp_steps() - 1u;

    CHECK(ellipsoid_ramp_step(1.0) == 0u);
    CHECK(ellipsoid_ramp_step(ramp_ceiling) == last);
    CHECK(ellipsoid_ramp_step(10.0 * ramp_ceiling) == last);
    CHECK(ellipsoid_ramp_step(std::nullopt) == last);
    CHECK(ellipsoid_ramp_step(std::sqrt(ramp_ceiling)) == (last + 1u) / 2u);
    CHECK_FALSE(tone_at(0u).equals(tone_at(last)));
}

TEST_CASE("a stencil told neither scale nor cap opens at the measured values", "[manipulator][drawing]")
{
    ellipsoid_stage headless(false);

    CHECK(headless.shown.ellipsoid_scale(jacobian_block::angular) == Catch::Approx(0.04));
    CHECK(headless.shown.ellipsoid_scale(jacobian_block::linear) == Catch::Approx(0.125));
    CHECK(headless.shown.force_cap_ratio() == Catch::Approx(2.4));
    CHECK(headless.shown.force_capped());
}
