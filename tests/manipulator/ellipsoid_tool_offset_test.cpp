#include "fixtures.h"
#include "drawn_columns.h"
#include "drawn_ellipsoids.h"

#include "robot/column_arrow.h"
#include "robot/manipulability.h"

#include "praxis/manipulator/loadable_robot_stencil.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

// A flange Jacobian whose blocks are not diagonal, so a rotation of the offset changes the
// decomposition of the linear block and not only its axes.
jacobian flange_columns()
{
    jacobian built = diagonal_blocks(Eigen::Vector3d(1.0, 0.6, 0.2));
    built(0, 1)    = 0.4;
    built(4, 0)    = -0.3;

    return built;
}

// The tool frame turned about no coordinate axis and moved off the flange along all three.
transform rotating_offset()
{
    transform offset         = transform::Identity();
    offset.block<3, 3>(0, 0) = turned_tool();
    offset.block<3, 1>(0, 3) = Eigen::Vector3d(0.05, 0.1, -0.02);

    return offset;
}

arm_snapshot under_the_offset()
{
    arm_snapshot seen     = published(refused(), refused(), Eigen::Vector3d(0.3, -0.2, 0.7));
    seen.tool_offset      = rotating_offset();
    seen.tool_orientation = rotation(turned_tool());
    seen.body_jacobian    = flange_columns();

    return seen;
}

}

TEST_CASE("under a rotating tool offset both body ellipsoids are taken from the Jacobian the body columns read", "[manipulator][drawing]")
{
    const arm_snapshot seen                   = under_the_offset();
    const expected<jacobian, refusal> columns = shown_jacobian(seen, jacobian_frame::body);
    REQUIRE(columns.has_value());
    const expected<manipulability_ellipsoid, refusal> linear  = ellipsoid_of(columns->bottomRows(3));
    const expected<manipulability_ellipsoid, refusal> angular = ellipsoid_of(columns->topRows(3));
    REQUIRE(linear.has_value());
    REQUIRE(angular.has_value());
    REQUIRE((linear->singular_values - ellipsoid_of(flange_columns().bottomRows(3))->singular_values).norm() > 1.0e-3);

    ellipsoid_stage headless;
    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::velocity);
    headless.shown.set_jacobian_frame(jacobian_frame::body);
    headless.put(seen);
    headless.draw();

    CHECK((body_extents(headless.body(jacobian_block::linear)) - linear_scale * linear->singular_values).norm() < read_back);
    CHECK((body_axes(headless.body(jacobian_block::linear)) - turned_tool() * linear->principal_axes).norm() < read_back);
    CHECK((body_axes(headless.body(jacobian_block::angular)) - turned_tool() * angular->principal_axes).norm() < read_back);
}
