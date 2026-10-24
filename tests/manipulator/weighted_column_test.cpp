#include "fixtures.h"
#include "drawn_columns.h"
#include "drawn_ellipsoids.h"

#include "captured_log.h"

#include "../presets/drawn_lines.h"

#include "praxis/manipulator/loadable_robot_stencil.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <threepp/core/Object3D.hpp>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

constexpr double built_head_length = 0.02;
constexpr double not_a_number      = std::numeric_limits<double>::quiet_NaN();
constexpr double unbounded         = std::numeric_limits<double>::infinity();

constexpr jacobian_block both_parts[] = {jacobian_block::angular, jacobian_block::linear};

const Eigen::Vector3d elsewhere(0.3, -0.2, 0.7);

// The shaft and the head end to end, or the head alone, shrunk, where the length leaves no shaft.
double drawn_length(threepp::Object3D *arrow)
{
    threepp::Object3D *const shaft = shaft_of(arrow);
    threepp::Object3D *const head  = head_of(arrow);
    REQUIRE(shaft != nullptr);
    REQUIRE(head != nullptr);
    if(shaft->visible)
        return static_cast<double>(shaft->scale.y) + built_head_length;

    return static_cast<double>(head->scale.y) * built_head_length;
}

void tell(column_stage &stage, bool weighted, const std::vector<double> &rates, std::size_t columns = 2u)
{
    REQUIRE(stage.shown.set_jacobian_columns(columns).has_value());
    stage.shown.set_columns_weighted(weighted);
    REQUIRE(stage.shown.set_joint_rates(rates).has_value());
    stage.draw();
}

// Each column stands as the reference's would at its own rate: undrawn at zero, otherwise from the
// same anchor, turned round where the rate is negative, at the rate's magnitude times its length.
void weighted_like(column_stage &weighted, column_stage &reference, const std::vector<double> &rates)
{
    for(std::size_t column = 0; column < rates.size(); ++column)
        for(const jacobian_block part : both_parts)
        {
            threepp::Object3D *const told = weighted.arrow(column, part);
            threepp::Object3D *const unit = reference.arrow(column, part);
            if(rates[column] == 0.0)
            {
                CHECK_FALSE(drawn(told));
                continue;
            }

            REQUIRE(drawn(told));
            CHECK((arrow_at(told) - arrow_at(unit)).norm() < placed_back);
            CHECK((arrow_along(told) - (rates[column] > 0.0 ? 1.0 : -1.0) * arrow_along(unit)).norm() < placed_back);
            CHECK(drawn_length(told) == Catch::Approx(std::abs(rates[column]) * drawn_length(unit)).margin(1e-6));
        }
}

// What declining the rates wrote to the log.
std::string declined(column_stage &stage, const std::vector<double> &rates)
{
    bool refused            = false;
    const std::string wrote = reported_by([&stage, &rates, &refused] { refused = !stage.shown.set_joint_rates(rates).has_value(); });
    CHECK(refused);

    return wrote;
}

// The unweighted placement written out from the published space Jacobian: each part at the space
// origin, along its own three rows, at their norm times that part's scale.
void stands_as_unweighted(column_stage &stage)
{
    const jacobian published = two_columns(1.0);
    for(std::size_t column = 0; column < 2u; ++column)
        for(const jacobian_block part : both_parts)
        {
            const Eigen::Vector3d rows     = published.block<3, 1>(part == jacobian_block::angular ? 0 : 3, static_cast<Eigen::Index>(column));
            const double scale             = part == jacobian_block::angular ? angular_column_scale : linear_column_scale;
            threepp::Object3D *const arrow = stage.arrow(column, part);
            REQUIRE(drawn(arrow));
            CHECK(arrow_at(arrow).norm() < placed_back);
            CHECK((arrow_along(arrow) - rows.normalized()).norm() < placed_back);
            CHECK(drawn_length(arrow) == Catch::Approx(rows.norm() * scale).margin(1e-6));
        }
}

// Compared exactly: the placement written is the reference's to the bit.
void same_standing(column_stage &told, column_stage &reference)
{
    for(std::size_t column = 0; column < 2u; ++column)
        for(const jacobian_block part : both_parts)
        {
            threepp::Object3D *const arrow = told.arrow(column, part);
            threepp::Object3D *const unit  = reference.arrow(column, part);
            REQUIRE(drawn(arrow));
            CHECK(shaft_stretch(arrow) == shaft_stretch(unit));
            CHECK(arrow->position == unit->position);
            CHECK(arrow->quaternion == unit->quaternion);
        }
}

}

TEST_CASE("columns told two rates and weighted stand at each rate times their unit length, the negative one turned round", "[manipulator][drawing]")
{
    column_stage reference;
    tell(reference, false, {});
    column_stage weighted;
    tell(weighted, true, {2.0, -0.5});

    weighted_like(weighted, reference, {2.0, -0.5});
}

TEST_CASE("a stencil told no rates, told rates unweighted or weighted with no rates stands every arrow exactly where an untold one does", "[manipulator][drawing]")
{
    column_stage reference;
    REQUIRE(reference.shown.set_jacobian_columns(2u).has_value());
    reference.draw();
    stands_as_unweighted(reference);

    column_stage untold, unweighted, bare, ones;
    tell(untold, false, {});
    tell(unweighted, false, {2.0, -0.5});
    tell(bare, true, {});
    tell(ones, true, {1.0, 1.0});

    same_standing(untold, reference);
    same_standing(unweighted, reference);
    same_standing(bare, reference);
    same_standing(ones, reference);
}

TEST_CASE("a zero rate of either sign leaves its column undrawn and rates a step either side of zero turn it round", "[manipulator][drawing]")
{
    column_stage reference, zero, negative_zero;
    tell(reference, false, {});
    tell(zero, true, {0.0, 1.0});
    tell(negative_zero, true, {-0.0, 1.0});
    weighted_like(zero, reference, {0.0, 1.0});
    weighted_like(negative_zero, reference, {-0.0, 1.0});

    const jacobian equal =
            of_columns({twist_of(Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 2.0, 0.0)), twist_of(Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 2.0, 0.0))});
    column_stage equal_reference, stepped;
    equal_reference.put(published_columns(equal, equal, Eigen::Vector3d(Eigen::Vector3d::Zero())));
    stepped.put(published_columns(equal, equal, Eigen::Vector3d(Eigen::Vector3d::Zero())));
    tell(equal_reference, false, {});
    tell(stepped, true, {1e-3, -1e-3});

    weighted_like(stepped, equal_reference, {1e-3, -1e-3});
    for(const jacobian_block part : both_parts)
        CHECK((arrow_along(stepped.arrow(0u, part)) + arrow_along(stepped.arrow(1u, part))).norm() < placed_back);
}

TEST_CASE("rate i weights column i, and two columns told the same rate keep an arrow each under its own name", "[manipulator][drawing]")
{
    column_stage reference, ordered, alike;
    tell(reference, false, {});
    tell(ordered, true, {3.0, 0.25});
    tell(alike, true, {2.0, 2.0});

    weighted_like(ordered, reference, {3.0, 0.25});
    weighted_like(alike, reference, {2.0, 2.0});
    for(const jacobian_block part : both_parts)
        CHECK(alike.arrow(0u, part)->name != alike.arrow(1u, part)->name);
}

TEST_CASE("rates of another width or that are not finite are declined by name and leave the arrows where they stood", "[manipulator][drawing]")
{
    column_stage before, refused;
    tell(before, true, {2.0, -0.5});
    tell(refused, true, {2.0, -0.5});

    const std::string wider = declined(refused, {1.0, 2.0, 3.0});
    CHECK(wider.find('3') != std::string::npos);
    CHECK(wider.find('2') != std::string::npos);
    for(const double unread : {not_a_number, unbounded, -unbounded})
        CHECK_FALSE(declined(refused, {1.0, unread}).empty());

    refused.draw();
    same_standing(refused, before);
}

TEST_CASE("an empty span clears the rates while weighting stays on, and rates told unweighted are weighted once the switch is turned", "[manipulator][drawing]")
{
    column_stage reference, cleared, waiting;
    tell(reference, false, {});
    tell(cleared, true, {2.0, -0.5});
    tell(waiting, false, {2.0, -0.5});

    REQUIRE(cleared.shown.set_joint_rates({}).has_value());
    cleared.draw();
    CHECK(cleared.shown.columns_weighted());
    same_standing(cleared, reference);

    same_standing(waiting, reference);
    waiting.shown.set_columns_weighted(true);
    waiting.draw();
    weighted_like(waiting, reference, {2.0, -0.5});
}

TEST_CASE("telling the column count again clears the rates", "[manipulator][drawing]")
{
    column_stage reference, retold;
    tell(reference, false, {});
    tell(retold, true, {2.0, -0.5});

    REQUIRE(retold.shown.set_jacobian_columns(2u).has_value());
    retold.draw();
    same_standing(retold, reference);

    REQUIRE(retold.shown.set_jacobian_columns(3u).has_value());
    CHECK(retold.shown.set_joint_rates(std::vector<double>{1.0, 2.0, 3.0}).has_value());
}

TEST_CASE("a one-column drawing takes one rate and declines two", "[manipulator][drawing]")
{
    const jacobian single = of_columns({twist_of(Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 2.0, 0.0))});
    column_stage reference, weighted;
    reference.put(published_columns(single, single, Eigen::Vector3d(Eigen::Vector3d::Zero())));
    weighted.put(published_columns(single, single, Eigen::Vector3d(Eigen::Vector3d::Zero())));
    tell(reference, false, {}, 1u);
    tell(weighted, true, {2.0}, 1u);

    CHECK_FALSE(declined(weighted, {1.0, 1.0}).empty());
    weighted.draw();
    weighted_like(weighted, reference, {2.0});
}

TEST_CASE("with the body Jacobian shown each column stands at the tool at its own rate's weight, and keeps it when the frame flips back", "[manipulator][drawing]")
{
    const Eigen::Matrix3d turned = turned_tool();
    const jacobian body          = two_columns(1.0);
    arm_snapshot seen            = published_columns(carried_into_space(turned, elsewhere, body), body, elsewhere);
    seen.tool_orientation        = rotation(turned);

    column_stage reference, weighted;
    for(column_stage *const stage : {&reference, &weighted})
    {
        stage->put(seen);
        stage->shown.set_jacobian_frame(jacobian_frame::body);
    }
    tell(reference, false, {});
    tell(weighted, true, {2.0, -0.5});

    CHECK((arrow_at(weighted.arrow(1u, jacobian_block::linear)) - elsewhere).norm() < placed_back);
    weighted_like(weighted, reference, {2.0, -0.5});

    for(column_stage *const stage : {&reference, &weighted})
    {
        stage->shown.set_jacobian_frame(jacobian_frame::space);
        stage->draw();
    }
    weighted_like(weighted, reference, {2.0, -0.5});
}

TEST_CASE("the column of the joint the drawing tells apart is weighted like every other", "[manipulator][drawing]")
{
    column_stage reference, weighted;
    for(column_stage *const stage : {&reference, &weighted})
        REQUIRE(stage->shown.set_selected_joint(1u).has_value());
    tell(reference, false, {});
    tell(weighted, true, {2.0, -0.5});

    weighted_like(weighted, reference, {2.0, -0.5});
}
