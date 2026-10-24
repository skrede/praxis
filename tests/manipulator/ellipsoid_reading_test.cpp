#include "fixtures.h"
#include "drawn_ellipsoids.h"

#include "../presets/drawn_lines.h"

#include "praxis/manipulator/loadable_robot_stencil.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/core/BufferGeometry.hpp>

#include <Eigen/Core>

#include <cmath>
#include <vector>
#include <cstddef>
#include <algorithm>

using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

// The principal axes are handed in as the identity, so the greatest magnitude on each of a body's
// own coordinate axes is that axis's drawn semi-axis.
Eigen::Vector3d extents(threepp::Object3D *drawn)
{
    REQUIRE(drawn != nullptr);
    const std::vector<float> &raw = drawn->geometry()->getAttribute<float>("position")->array();

    Eigen::Vector3d worst = Eigen::Vector3d::Zero();
    for(std::size_t at = 0; at + 2 < raw.size(); at += 3)
        for(Eigen::Index axis = 0; axis < 3; ++axis)
            worst[axis] = std::max(worst[axis], std::abs(static_cast<double>(raw[at + static_cast<std::size_t>(axis)])));

    return worst;
}

}

TEST_CASE("in the velocity reading a drawn semi-axis is the block's own scale times its singular value", "[manipulator][drawing]")
{
    ellipsoid_stage headless;
    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::velocity);

    const Eigen::Vector3d values(1.0, 0.6, 0.2);
    headless.put(published(decomposed_both(values), refused(), Eigen::Vector3d(Eigen::Vector3d::Zero())));
    headless.draw();

    CHECK((extents(headless.body(jacobian_block::angular)) - angular_scale * values).norm() < read_back);
    CHECK((extents(headless.body(jacobian_block::linear)) - linear_scale * values).norm() < read_back);
}

TEST_CASE("in the force reading a drawn semi-axis is the scale over the singular value, cut at the cap, with a line where the cut binds", "[manipulator][drawing]")
{
    ellipsoid_stage headless;
    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::force);

    const Eigen::Vector3d values(4.0, 1.0, 0.5);
    headless.put(published(decomposed_both(values), refused(), Eigen::Vector3d(Eigen::Vector3d::Zero())));
    headless.draw();

    for(Eigen::Index axis = 0; axis < 3; ++axis)
    {
        const double asked = linear_scale / values[axis];
        CHECK(std::abs(extents(headless.body(jacobian_block::linear))[axis] - std::min(asked, linear_cap)) < read_back);
        for(const bool forward : {true, false})
            CHECK(drawn(headless.line(jacobian_block::linear, static_cast<std::size_t>(axis), forward)) == (asked > linear_cap));
    }
}

TEST_CASE("lifting the cap leaves the semi-axis at the scale over the singular value and every continuation line undrawn", "[manipulator][drawing]")
{
    ellipsoid_stage headless;
    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::force);
    headless.shown.set_force_capped(false);

    const Eigen::Vector3d values(4.0, 1.0, 0.5);
    headless.put(published(decomposed_both(values), refused(), Eigen::Vector3d(Eigen::Vector3d::Zero())));
    headless.draw();

    const Eigen::Vector3d asked(linear_scale / values.array());
    CHECK((extents(headless.body(jacobian_block::linear)) - asked).norm() < read_back);
    CHECK(headless.shown.force_cap_ratio() == cap_ratio);
    for(std::size_t axis = 0; axis < 3u; ++axis)
        for(const bool forward : {true, false})
            CHECK_FALSE(drawn(headless.line(jacobian_block::linear, axis, forward)));
}

TEST_CASE("an axis sitting exactly on the cap is cut but draws no continuation line", "[manipulator][drawing]")
{
    ellipsoid_stage headless;
    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::force);

    const Eigen::Vector3d values(linear_scale / linear_cap, 1.0, 0.5);
    headless.put(published(decomposed_both(values), refused(), Eigen::Vector3d(Eigen::Vector3d::Zero())));
    headless.draw();

    CHECK(std::abs(extents(headless.body(jacobian_block::linear))[0] - linear_cap) < read_back);
    for(const bool forward : {true, false})
        CHECK_FALSE(drawn(headless.line(jacobian_block::linear, 0u, forward)));
}

TEST_CASE("a force reading whose smallest singular value is zero draws nothing for that block and leaves the velocity reading drawable", "[manipulator][drawing]")
{
    ellipsoid_stage headless;
    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::force);

    const Eigen::Vector3d values(1.0, 0.5, 0.0);
    headless.put(published(decomposed_both(values), refused(), Eigen::Vector3d(Eigen::Vector3d::Zero())));
    headless.draw();

    CHECK_FALSE(drawn(headless.body(jacobian_block::linear)));
    for(std::size_t axis = 0; axis < 3u; ++axis)
        for(const bool forward : {true, false})
            CHECK_FALSE(drawn(headless.line(jacobian_block::linear, axis, forward)));

    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::velocity);
    headless.draw();

    CHECK(drawn(headless.body(jacobian_block::linear)));
    CHECK((extents(headless.body(jacobian_block::linear)) - linear_scale * values).norm() < read_back);
}

TEST_CASE("a publication whose blocks are refusals leaves both bodies undrawn and asks nothing of the drawing", "[manipulator][drawing]")
{
    ellipsoid_stage headless;
    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::velocity);
    headless.put(published(refused(), refused(), Eigen::Vector3d(Eigen::Vector3d::Zero())));
    headless.draw();

    CHECK_FALSE(drawn(headless.body(jacobian_block::angular)));
    CHECK_FALSE(drawn(headless.body(jacobian_block::linear)));
    CHECK(drawn(headless.scene->getObjectByName<threepp::Object3D>(loadable_robot_stencil::chain_name())));
}

TEST_CASE("a publication carrying no tool position leaves both bodies undrawn", "[manipulator][drawing]")
{
    ellipsoid_stage headless;
    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::velocity);
    headless.put(published(decomposed_both(Eigen::Vector3d(1.0, 0.6, 0.2)), decomposed_both(Eigen::Vector3d(1.0, 0.6, 0.2)), praxis::unexpected(praxis::refusal::not_implemented)));
    headless.draw();

    CHECK_FALSE(drawn(headless.body(jacobian_block::angular)));
    CHECK_FALSE(drawn(headless.body(jacobian_block::linear)));
}

TEST_CASE("the Jacobian the stencil is told to show is the one both bodies are taken from", "[manipulator][drawing]")
{
    ellipsoid_stage headless;
    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::velocity);

    const Eigen::Vector3d in_space(1.0, 0.6, 0.2);
    const Eigen::Vector3d in_body(2.0, 1.2, 0.4);
    headless.put(published(decomposed_both(in_space), decomposed_both(in_body), Eigen::Vector3d(Eigen::Vector3d::Zero())));
    headless.draw();

    CHECK(headless.shown.jacobian_frame_shown() == jacobian_frame::space);
    CHECK((extents(headless.body(jacobian_block::linear)) - linear_scale * in_space).norm() < read_back);

    headless.shown.set_jacobian_frame(jacobian_frame::body);
    headless.draw();

    CHECK(headless.shown.jacobian_frame_shown() == jacobian_frame::body);
    CHECK((extents(headless.body(jacobian_block::linear)) - linear_scale * in_body).norm() < read_back);
}

TEST_CASE("one multiple cuts the two blocks at different lengths, each at its own scale", "[manipulator][drawing]")
{
    ellipsoid_stage headless;
    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::force);

    headless.put(published(decomposed_both(Eigen::Vector3d(4.0, 1.0, 0.5)), refused(), Eigen::Vector3d(Eigen::Vector3d::Zero())));
    headless.draw();

    CHECK(std::abs(extents(headless.body(jacobian_block::angular))[2] - angular_cap) < read_back);
    CHECK(std::abs(extents(headless.body(jacobian_block::linear))[2] - linear_cap) < read_back);
}

TEST_CASE("moving one block's scale moves that block's cut and leaves the other's standing", "[manipulator][drawing]")
{
    ellipsoid_stage headless;
    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::force);

    headless.put(published(decomposed_both(Eigen::Vector3d(4.0, 1.0, 0.5)), refused(), Eigen::Vector3d(Eigen::Vector3d::Zero())));
    headless.shown.set_ellipsoid_scale(jacobian_block::linear, 2.0 * linear_scale);
    headless.draw();

    CHECK(std::abs(extents(headless.body(jacobian_block::linear))[2] - 2.0 * linear_cap) < read_back);
    CHECK(std::abs(extents(headless.body(jacobian_block::angular))[2] - angular_cap) < read_back);
}
