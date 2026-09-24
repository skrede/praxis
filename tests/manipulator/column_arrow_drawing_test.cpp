#include "fixtures.h"
#include "drawn_columns.h"
#include "robot/column_arrow.h"
#include "drawn_chain.h"
#include "drawn_ellipsoids.h"

#include "../presets/drawn_lines.h"

#include "praxis/manipulator/loadable_robot_stencil.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

const Eigen::Vector3d elsewhere(0.3, -0.2, 0.7);

// What the geometry of one arrow is built at: the diameter of its shaft, the length of its head and
// the radius of that head, all in metres.
constexpr double built_shaft_girth = 0.008;
constexpr double built_head_length = 0.02;
constexpr double built_head_radius = 0.01;

}

TEST_CASE("telling a column count raises two arrows per column, all under one subtree of the decoration", "[manipulator][drawing]")
{
    column_stage headless;
    CHECK(headless.arrow(0u, jacobian_block::angular) == nullptr);

    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.draw();

    for(std::size_t column = 0; column < 2u; ++column)
        for(const jacobian_block part : {jacobian_block::angular, jacobian_block::linear})
            CHECK(drawn(headless.arrow(column, part)));

    CHECK(headless.arrow(0u, jacobian_block::angular)->parent == headless.arrow(1u, jacobian_block::angular)->parent);
    CHECK(headless.arrow(0u, jacobian_block::angular)->parent != headless.arrow(0u, jacobian_block::linear)->parent);
    CHECK(headless.arrow(0u, jacobian_block::angular)->parent->parent == headless.arrow(1u, jacobian_block::linear)->parent->parent);
    CHECK(headless.arrow(2u, jacobian_block::angular) == nullptr);
}

TEST_CASE("a column count of none is declined and leaves the arrows already standing where they were", "[manipulator][drawing]")
{
    column_stage headless;
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.draw();

    CHECK_FALSE(headless.shown.set_jacobian_columns(0u).has_value());
    headless.draw();

    CHECK(drawn(headless.arrow(0u, jacobian_block::angular)));
    CHECK(drawn(headless.arrow(1u, jacobian_block::linear)));
}

TEST_CASE("telling a second count replaces what stood with two arrows per column of the new count", "[manipulator][drawing]")
{
    column_stage headless;
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.draw();

    REQUIRE(headless.shown.set_jacobian_columns(3u).has_value());
    headless.draw();

    CHECK(headless.arrow(2u, jacobian_block::linear) != nullptr);
    CHECK(headless.arrow(3u, jacobian_block::angular) == nullptr);
}

TEST_CASE("with the body Jacobian shown every arrow stands at the published tool position", "[manipulator][drawing]")
{
    column_stage headless;
    headless.shown.set_jacobian_frame(jacobian_frame::body);
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.put(published_columns(two_columns(1.0), two_columns(1.0), elsewhere));
    headless.draw();

    for(std::size_t column = 0; column < 2u; ++column)
        for(const jacobian_block part : {jacobian_block::angular, jacobian_block::linear})
            CHECK((arrow_at(headless.arrow(column, part)) - elsewhere).norm() < placed_back);
}

TEST_CASE("with the space Jacobian shown every arrow stands at the space origin whatever the tool position is", "[manipulator][drawing]")
{
    column_stage headless;
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.put(published_columns(two_columns(1.0), two_columns(1.0), elsewhere));
    headless.draw();

    for(std::size_t column = 0; column < 2u; ++column)
        for(const jacobian_block part : {jacobian_block::angular, jacobian_block::linear})
            CHECK(arrow_at(headless.arrow(column, part)).norm() < placed_back);
}

TEST_CASE("flipping the one Jacobian field moves every arrow and moves both ellipsoids with them", "[manipulator][drawing]")
{
    column_stage headless;
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::velocity);
    headless.shown.set_ellipsoid_scale(jacobian_block::angular, angular_scale);
    headless.shown.set_ellipsoid_scale(jacobian_block::linear, linear_scale);
    headless.put(
            published_columns(two_columns(1.0), two_columns_exchanged(), elsewhere, decomposed_both(Eigen::Vector3d(1.0, 0.6, 0.2)), decomposed_both(Eigen::Vector3d(0.5, 0.3, 0.1))));
    headless.draw();

    const Eigen::Vector3d in_space = arrow_at(headless.arrow(0u, jacobian_block::linear));
    const Eigen::Vector3d spread   = body_extents(headless.body(jacobian_block::linear));
    CHECK((arrow_along(headless.arrow(0u, jacobian_block::angular)) - Eigen::Vector3d::UnitZ()).norm() < placed_back);

    headless.shown.set_jacobian_frame(jacobian_frame::body);
    headless.draw();

    CHECK((arrow_along(headless.arrow(0u, jacobian_block::angular)) - Eigen::Vector3d::UnitY()).norm() < placed_back);
    CHECK((arrow_at(headless.arrow(0u, jacobian_block::linear)) - in_space).norm() > 0.1);
    CHECK((arrow_at(headless.arrow(1u, jacobian_block::angular)) - elsewhere).norm() < placed_back);
    CHECK((body_extents(headless.body(jacobian_block::linear)) - spread).norm() > placed_back);
    CHECK((body_extents(headless.body(jacobian_block::angular)) - 0.5 * angular_scale * Eigen::Vector3d(1.0, 0.6, 0.2)).norm() < read_back);
}

TEST_CASE("an arrow points along its own three rows of its own column and is as long as their norm times that part's scale", "[manipulator][drawing]")
{
    column_stage headless;
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.put(published_columns(two_columns(1.0), two_columns(1.0), Eigen::Vector3d(Eigen::Vector3d::Zero())));
    headless.draw();

    CHECK((arrow_along(headless.arrow(0u, jacobian_block::angular)) - Eigen::Vector3d::UnitZ()).norm() < placed_back);
    CHECK((arrow_along(headless.arrow(0u, jacobian_block::linear)) - Eigen::Vector3d::UnitY()).norm() < placed_back);
    CHECK((arrow_along(headless.arrow(1u, jacobian_block::angular)) - Eigen::Vector3d::UnitX()).norm() < placed_back);
    CHECK((arrow_along(headless.arrow(1u, jacobian_block::linear)) - Eigen::Vector3d::UnitZ()).norm() < placed_back);

    // The head is the same length at every arrow, so it is taken off the one arrow whose length the
    // published column and the told scale name outright and the rest are read against it.
    const double head = 2.0 * linear_column_scale - shaft_stretch(headless.arrow(0u, jacobian_block::linear));

    CHECK(shaft_stretch(headless.arrow(1u, jacobian_block::linear)) + head == Catch::Approx(3.0 * linear_column_scale));
    CHECK(shaft_stretch(headless.arrow(0u, jacobian_block::angular)) + head == Catch::Approx(angular_column_scale));
    CHECK(shaft_stretch(headless.arrow(1u, jacobian_block::angular)) + head == Catch::Approx(angular_column_scale));
}

TEST_CASE("a column part of no magnitude leaves that one arrow undrawn and the column's other arrow standing", "[manipulator][drawing]")
{
    column_stage headless;
    REQUIRE(headless.shown.set_jacobian_columns(1u).has_value());

    const jacobian spun = of_columns({twist_of(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero())});
    const jacobian slid = of_columns({twist_of(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX())});
    headless.put(published_columns(spun, spun, Eigen::Vector3d(Eigen::Vector3d::Zero())));
    headless.draw();

    CHECK(drawn(headless.arrow(0u, jacobian_block::angular)));
    CHECK_FALSE(drawn(headless.arrow(0u, jacobian_block::linear)));

    headless.put(published_columns(slid, slid, Eigen::Vector3d(Eigen::Vector3d::Zero())));
    headless.draw();

    CHECK_FALSE(drawn(headless.arrow(0u, jacobian_block::angular)));
    CHECK(drawn(headless.arrow(0u, jacobian_block::linear)));
}

TEST_CASE("a published Jacobian of another width than the count told leaves every arrow undrawn", "[manipulator][drawing]")
{
    column_stage headless;
    REQUIRE(headless.shown.set_jacobian_columns(3u).has_value());
    headless.put(published_columns(two_columns(1.0), two_columns(1.0), Eigen::Vector3d(Eigen::Vector3d::Zero())));
    headless.draw();

    for(std::size_t column = 0; column < 3u; ++column)
        for(const jacobian_block part : {jacobian_block::angular, jacobian_block::linear})
            CHECK_FALSE(drawn(headless.arrow(column, part)));
}

TEST_CASE("a Jacobian the arm declines to publish leaves every arrow undrawn", "[manipulator][drawing]")
{
    column_stage headless;
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.put(published_columns(praxis::unexpected(refusal::not_implemented), two_columns(1.0), Eigen::Vector3d(Eigen::Vector3d::Zero())));
    headless.draw();

    CHECK_FALSE(drawn(headless.arrow(0u, jacobian_block::angular)));
    CHECK_FALSE(drawn(headless.arrow(1u, jacobian_block::linear)));
}

TEST_CASE("a publication carrying no tool position leaves the body columns undrawn and the space columns standing", "[manipulator][drawing]")
{
    column_stage headless;
    headless.shown.set_jacobian_frame(jacobian_frame::body);
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.put(published_columns(two_columns(1.0), two_columns(1.0), praxis::unexpected(refusal::not_implemented)));
    headless.draw();

    CHECK_FALSE(drawn(headless.arrow(0u, jacobian_block::linear)));

    headless.shown.set_jacobian_frame(jacobian_frame::space);
    headless.draw();

    CHECK(drawn(headless.arrow(0u, jacobian_block::linear)));
    CHECK(arrow_at(headless.arrow(1u, jacobian_block::angular)).norm() < placed_back);
}

TEST_CASE("an arm too narrow for a decomposition still draws the columns of the Jacobian it publishes", "[manipulator][drawing]")
{
    column_stage headless;
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::velocity);
    headless.put(published_columns(two_columns(1.0), two_columns(1.0), Eigen::Vector3d(Eigen::Vector3d::Zero())));
    headless.draw();

    CHECK_FALSE(drawn(headless.body(jacobian_block::linear)));
    for(std::size_t column = 0; column < 2u; ++column)
        for(const jacobian_block part : {jacobian_block::angular, jacobian_block::linear})
            CHECK(drawn(headless.arrow(column, part)));
}

TEST_CASE("every combination of the ten switches leaves the scene in the state those ten name", "[manipulator][drawing]")
{
    column_stage headless;
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::velocity);
    REQUIRE(headless.shown.set_solution_figures(std::vector<joint_vector>{configuration(0.2, 0.4)}).has_value());
    headless.put(published_columns(two_columns(1.0), two_columns(1.0), Eigen::Vector3d(Eigen::Vector3d::Zero()), decomposed_both(Eigen::Vector3d(1.0, 0.6, 0.2)),
                                   decomposed_both(Eigen::Vector3d(1.0, 0.6, 0.2))));

    for(int combination = 0; combination < 1024; ++combination)
    {
        const bool meshes         = (combination & 1) != 0;
        const bool axes           = (combination & 2) != 0;
        const bool stick          = (combination & 4) != 0;
        const bool path           = (combination & 8) != 0;
        const bool figures        = (combination & 16) != 0;
        const bool angular        = (combination & 32) != 0;
        const bool linear         = (combination & 64) != 0;
        const bool columns        = (combination & 128) != 0;
        const bool angular_arrows = (combination & 256) != 0;
        const bool linear_arrows  = (combination & 512) != 0;

        headless.shown.set_meshes_shown(meshes);
        headless.shown.set_decoration_shown(axes);
        headless.shown.set_chain_shown(stick);
        REQUIRE(headless.shown.set_pose_path_shown("recorded", path).has_value());
        headless.shown.set_solution_figures_shown(figures);
        headless.shown.set_angular_ellipsoid_shown(angular);
        headless.shown.set_linear_ellipsoid_shown(linear);
        headless.shown.set_jacobian_columns_shown(columns);
        headless.shown.set_column_part_shown(jacobian_block::angular, angular_arrows);
        headless.shown.set_column_part_shown(jacobian_block::linear, linear_arrows);
        headless.draw();

        CHECK(drawn(rendered_arm(*headless.scene)) == meshes);
        CHECK(drawn(first_line_under(*headless.scene, loadable_robot_stencil::joint_axis_name(0))) == axes);
        CHECK(drawn(chain_node(*headless.scene, loadable_robot_stencil::chain_name())) == stick);
        CHECK(drawn(first_line_under(*headless.scene, loadable_robot_stencil::pose_path_name("recorded"))) == path);
        CHECK(drawn(chain_node(*headless.scene, loadable_robot_stencil::solution_figure_name(0))) == figures);
        CHECK(drawn(headless.body(jacobian_block::angular)) == angular);
        CHECK(drawn(headless.body(jacobian_block::linear)) == linear);
        CHECK(drawn(headless.arrow(0u, jacobian_block::angular)) == (columns && angular_arrows));
        CHECK(drawn(headless.arrow(1u, jacobian_block::linear)) == (columns && linear_arrows));
    }
}

TEST_CASE("an arrow of either part is built at the girth and the head its own geometry carries", "[manipulator][drawing]")
{
    column_stage headless;
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.draw();

    for(const jacobian_block part : {jacobian_block::angular, jacobian_block::linear})
    {
        const Eigen::Vector3d shaft = body_extents(shaft_of(headless.arrow(0u, part)));
        const Eigen::Vector3d head  = body_extents(head_of(headless.arrow(0u, part)));

        CHECK(2.0 * shaft.x() == Catch::Approx(built_shaft_girth).margin(read_back));
        CHECK(2.0 * shaft.z() == Catch::Approx(built_shaft_girth).margin(read_back));
        CHECK(2.0 * head.y() == Catch::Approx(built_head_length).margin(read_back));
        CHECK(head.x() == Catch::Approx(built_head_radius).margin(read_back));
        CHECK(head.z() == Catch::Approx(built_head_radius).margin(read_back));
    }
}

TEST_CASE("the angular part of a column stands the same way whichever Jacobian of one configuration is shown", "[manipulator][drawing]")
{
    const Eigen::Matrix3d turned = turned_tool();
    const jacobian body          = two_columns(1.0);

    arm_snapshot seen     = published_columns(carried_into_space(turned, elsewhere, body), body, elsewhere);
    seen.tool_orientation = rotation(turned);

    column_stage headless;
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.put(seen);
    headless.draw();

    const Eigen::Vector3d in_space = arrow_along(headless.arrow(0u, jacobian_block::angular));

    headless.shown.set_jacobian_frame(jacobian_frame::body);
    headless.draw();

    CHECK((arrow_along(headless.arrow(0u, jacobian_block::angular)) - in_space).norm() < placed_back);
    CHECK((arrow_at(headless.arrow(0u, jacobian_block::angular)) - elsewhere).norm() < placed_back);
}

TEST_CASE("a body Jacobian's columns are not stood at all while the tool's own orientation is a refusal", "[manipulator][drawing]")
{
    arm_snapshot seen     = published_columns(two_columns(1.0), two_columns(1.0), elsewhere);
    seen.tool_orientation = unexpected(refusal::not_implemented);

    column_stage headless;
    headless.shown.set_jacobian_frame(jacobian_frame::body);
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.put(seen);
    headless.draw();

    CHECK_FALSE(headless.arrow(0u, jacobian_block::angular)->visible);
    CHECK_FALSE(headless.arrow(1u, jacobian_block::linear)->visible);

    headless.shown.set_jacobian_frame(jacobian_frame::space);
    headless.draw();

    CHECK(headless.arrow(0u, jacobian_block::angular)->visible);
}

// The selection is the one the decoration already answers, so no case names a joint for the columns
// that the axes and the chain are not also about.
TEST_CASE("the joint the drawing tells apart tells its own column apart, and every other column is dimmed", "[manipulator][drawing]")
{
    column_stage headless;
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.draw();

    for(std::size_t column = 0; column < 2u; ++column)
        for(const jacobian_block part : {jacobian_block::angular, jacobian_block::linear})
            CHECK(arrow_tone(headless.arrow(column, part)) == column_tone(part, false));

    REQUIRE(headless.shown.set_selected_joint(0u).has_value());
    headless.draw();

    for(const jacobian_block part : {jacobian_block::angular, jacobian_block::linear})
    {
        CHECK(arrow_tone(headless.arrow(0u, part)) == column_tone(part, false));
        CHECK(arrow_tone(headless.arrow(1u, part)) == column_tone(part, true));
    }

    REQUIRE(headless.shown.set_selected_joint(1u).has_value());
    headless.draw();

    for(const jacobian_block part : {jacobian_block::angular, jacobian_block::linear})
    {
        CHECK(arrow_tone(headless.arrow(0u, part)) == column_tone(part, true));
        CHECK(arrow_tone(headless.arrow(1u, part)) == column_tone(part, false));
    }

    headless.shown.clear_selected_joint();
    headless.draw();

    for(std::size_t column = 0; column < 2u; ++column)
        for(const jacobian_block part : {jacobian_block::angular, jacobian_block::linear})
            CHECK(arrow_tone(headless.arrow(column, part)) == column_tone(part, false));
}

// A selection that stands while a width is told would otherwise raise arrows in the tone of a column
// the drawing is not about.
TEST_CASE("a column count told while a joint is told apart raises every arrow in the tone that selection names", "[manipulator][drawing]")
{
    column_stage headless;
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    REQUIRE(headless.shown.set_selected_joint(1u).has_value());
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.draw();

    for(const jacobian_block part : {jacobian_block::angular, jacobian_block::linear})
    {
        CHECK(arrow_tone(headless.arrow(0u, part)) == column_tone(part, true));
        CHECK(arrow_tone(headless.arrow(1u, part)) == column_tone(part, false));
    }
}

// A selection the drawing declines leaves the tones where they were, as it leaves the axes and the
// chain.
TEST_CASE("a joint the drawing carries no screw for is declined and no column changes tone", "[manipulator][drawing]")
{
    column_stage headless;
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    REQUIRE(headless.shown.set_selected_joint(0u).has_value());
    headless.draw();

    CHECK_FALSE(headless.shown.set_selected_joint(2u).has_value());
    headless.draw();

    for(const jacobian_block part : {jacobian_block::angular, jacobian_block::linear})
    {
        CHECK(arrow_tone(headless.arrow(0u, part)) == column_tone(part, false));
        CHECK(arrow_tone(headless.arrow(1u, part)) == column_tone(part, true));
    }
}

// Either part of every column is withheld on its own, beneath the switch over the columns at all.
TEST_CASE("a part withheld takes that part's arrow from every column and leaves the other part's standing", "[manipulator][drawing]")
{
    column_stage headless;
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.shown.set_column_part_shown(jacobian_block::angular, false);
    headless.draw();

    CHECK_FALSE(drawn(headless.arrow(0u, jacobian_block::angular)));
    CHECK_FALSE(drawn(headless.arrow(1u, jacobian_block::angular)));
    CHECK(drawn(headless.arrow(0u, jacobian_block::linear)));
    CHECK(drawn(headless.arrow(1u, jacobian_block::linear)));

    headless.shown.set_column_part_shown(jacobian_block::angular, true);
    headless.shown.set_jacobian_columns_shown(false);
    headless.draw();

    CHECK_FALSE(drawn(headless.arrow(0u, jacobian_block::angular)));
    CHECK_FALSE(drawn(headless.arrow(0u, jacobian_block::linear)));

    headless.shown.set_jacobian_columns_shown(true);
    headless.draw();

    CHECK(drawn(headless.arrow(0u, jacobian_block::angular)));
    CHECK(drawn(headless.arrow(0u, jacobian_block::linear)));
}

// A width told again replaces the arrows, so a part withheld before it must still be withheld after:
// the switch is on the subtree the arrows are raised under rather than on the arrows.
TEST_CASE("a part withheld stays withheld through a column count told again", "[manipulator][drawing]")
{
    column_stage headless;
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.shown.set_column_part_shown(jacobian_block::linear, false);
    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.draw();

    CHECK(drawn(headless.arrow(1u, jacobian_block::angular)));
    CHECK_FALSE(drawn(headless.arrow(1u, jacobian_block::linear)));
}
