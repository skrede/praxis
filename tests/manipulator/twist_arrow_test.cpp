#include "fixtures.h"
#include "drawn_columns.h"
#include "robot/column_arrow.h"
#include "drawn_ellipsoids.h"

#include "../presets/drawn_lines.h"

#include "praxis/manipulator/loadable_robot_stencil.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/core/Object3D.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

const Eigen::Vector3d elsewhere(0.3, -0.2, 0.7);

constexpr double head_length = 0.02;

constexpr jacobian_block both_parts[] = {jacobian_block::angular, jacobian_block::linear};

const std::vector<double> told_rates{2.0, -0.5};

// A drawn vector is read back off a placement held in single precision.
constexpr double vector_read_back = 1.0e-4;

threepp::Object3D *twist_arrow(column_stage &stage, jacobian_block part)
{
    return stage.scene->getObjectByName<threepp::Object3D>(loadable_robot_stencil::twist_name(part));
}

// The arrow's direction times its whole drawn length, shaft and head end to end.
Eigen::Vector3d drawn_vector(threepp::Object3D *arrow)
{
    return arrow_along(arrow) * (shaft_stretch(arrow) + head_length);
}

Eigen::Matrix<double, 6, 1> times_rates(const jacobian &taken)
{
    return taken * Eigen::Map<const Eigen::VectorXd>(told_rates.data(), static_cast<Eigen::Index>(told_rates.size()));
}

Eigen::Matrix4d posed(const Eigen::Matrix3d &turn, const Eigen::Vector3d &at)
{
    Eigen::Matrix4d pose   = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = turn;
    pose.block<3, 1>(0, 3) = at;

    return pose;
}

// An arm whose body Jacobian is taken at the flange, the space one its adjoint at the flange pose,
// with the tool hung at the offset beyond it.
arm_snapshot hung_from(const Eigen::Matrix4d &flange, const Eigen::Matrix4d &offset)
{
    const Eigen::Matrix4d tool = flange * offset;
    const Eigen::Vector3d at   = tool.block<3, 1>(0, 3);

    arm_snapshot seen       = published_columns(carried_into_space(flange.block<3, 3>(0, 0), flange.block<3, 1>(0, 3), two_columns(1.0)), two_columns(1.0), at);
    seen.tool_offset        = offset;
    seen.tool_pose          = tool;
    seen.flange_pose        = flange;
    seen.flange_position    = Eigen::Vector3d(flange.block<3, 1>(0, 3));
    seen.tool_orientation   = rotation(tool.block<3, 3>(0, 0));
    seen.flange_orientation = rotation(flange.block<3, 3>(0, 0));

    return seen;
}

const Eigen::Matrix4d tool_hung = posed(Eigen::Matrix3d(Eigen::AngleAxisd(0.9, Eigen::Vector3d(-2.0, 1.0, 0.5).normalized())), Eigen::Vector3d(0.05, -0.12, 0.2));

// What the column's joint gives the tool point: the angular part, and the space linear part with the
// angular part crossed into the tool position. Lynch & Park, Modern Robotics, section 5.1.
Eigen::Matrix<double, 6, 1> at_tool_point(const jacobian &space, Eigen::Index column, const Eigen::Vector3d &tool)
{
    const Eigen::Vector3d omega = space.block<3, 1>(0, column);

    return twist_of(omega, space.block<3, 1>(3, column) + omega.cross(tool));
}

void tell(column_stage &stage, const arm_snapshot &seen, jacobian_frame frame = jacobian_frame::space)
{
    REQUIRE(stage.shown.set_jacobian_columns(2u).has_value());
    stage.put(seen);
    REQUIRE(stage.shown.set_joint_rates(told_rates).has_value());
    stage.shown.set_jacobian_frame(frame);
    stage.shown.set_twist_shown(true);
    stage.draw();
}

void drawn_as(column_stage &stage, bool angular, bool linear)
{
    stage.draw();
    CHECK(drawn(twist_arrow(stage, jacobian_block::angular)) == angular);
    CHECK(drawn(twist_arrow(stage, jacobian_block::linear)) == linear);
}

void hidden_once(const arm_snapshot &refused_one, jacobian_frame frame)
{
    column_stage headless;
    tell(headless, published_columns(two_columns(1.0), two_columns(1.0), elsewhere), frame);
    drawn_as(headless, true, true);

    headless.put(refused_one);
    drawn_as(headless, false, false);
}

}

TEST_CASE("the twist of told rates stands at the tool, its angular arrow J's angular rows times the rates and its linear arrow the tool point's velocity", "[manipulator][drawing]")
{
    column_stage headless;
    tell(headless, published_columns(two_columns(1.0), two_columns(1.0), elsewhere));

    const Eigen::Matrix<double, 6, 1> twist = times_rates(two_columns(1.0));
    const Eigen::Vector3d omega             = twist.head<3>();
    const Eigen::Vector3d moving            = twist.tail<3>() + omega.cross(elsewhere);

    threepp::Object3D *const angular = twist_arrow(headless, jacobian_block::angular);
    threepp::Object3D *const linear  = twist_arrow(headless, jacobian_block::linear);
    REQUIRE(drawn(angular));
    REQUIRE(drawn(linear));
    CHECK((arrow_at(angular) - elsewhere).norm() < placed_back);
    CHECK((arrow_at(linear) - elsewhere).norm() < placed_back);
    CHECK((drawn_vector(angular) - omega * angular_column_scale).norm() < vector_read_back);
    CHECK((drawn_vector(linear) - moving * linear_column_scale).norm() < vector_read_back);
}

TEST_CASE("with the body Jacobian shown the twist stands at the tool, both parts carried into space by the tool's orientation", "[manipulator][drawing]")
{
    column_stage headless;
    tell(headless, hung_from(posed(turned_tool(), elsewhere), Eigen::Matrix4d::Identity()), jacobian_frame::body);

    const Eigen::Matrix<double, 6, 1> twist = times_rates(two_columns(1.0));
    for(const jacobian_block part : both_parts)
        REQUIRE(drawn(twist_arrow(headless, part)));
    CHECK((arrow_at(twist_arrow(headless, jacobian_block::linear)) - elsewhere).norm() < placed_back);
    CHECK((drawn_vector(twist_arrow(headless, jacobian_block::angular)) - turned_tool() * twist.head<3>() * angular_column_scale).norm() < vector_read_back);
    CHECK((drawn_vector(twist_arrow(headless, jacobian_block::linear)) - turned_tool() * twist.tail<3>() * linear_column_scale).norm() < vector_read_back);
}

// The space Jacobian published is the adjoint of the body one at the flange pose, the frame the body
// Jacobian is taken in: J_s = [Ad_T] J_b.
TEST_CASE("the twist stands as the same two vectors whichever Jacobian of one configuration is shown", "[manipulator][drawing]")
{
    column_stage headless;
    tell(headless, hung_from(posed(turned_tool(), elsewhere), Eigen::Matrix4d::Identity()));
    headless.shown.set_columns_weighted(true);
    headless.draw();

    std::vector<Eigen::Vector3d> in_space;
    for(const jacobian_block part : both_parts)
        in_space.push_back(drawn_vector(twist_arrow(headless, part)));
    const Eigen::Vector3d columns_at_origin = drawn_vector(headless.arrow(0u, jacobian_block::linear)) + drawn_vector(headless.arrow(1u, jacobian_block::linear));
    CHECK((columns_at_origin - in_space[1]).norm() > 1e-3);

    headless.shown.set_jacobian_frame(jacobian_frame::body);
    headless.draw();
    for(const jacobian_block part : both_parts)
    {
        CHECK((arrow_at(twist_arrow(headless, part)) - elsewhere).norm() < placed_back);
        CHECK((drawn_vector(twist_arrow(headless, part)) - in_space[static_cast<std::size_t>(part)]).norm() < vector_read_back);
    }
}

TEST_CASE("in the body frame the twist is the sum of the weighted columns standing at the tool", "[manipulator][drawing]")
{
    column_stage headless;
    tell(headless, hung_from(posed(turned_tool(), elsewhere), Eigen::Matrix4d::Identity()), jacobian_frame::body);
    headless.shown.set_columns_weighted(true);
    headless.draw();

    for(const jacobian_block part : both_parts)
    {
        const Eigen::Vector3d summed = drawn_vector(headless.arrow(0u, part)) + drawn_vector(headless.arrow(1u, part));
        CHECK((drawn_vector(twist_arrow(headless, part)) - summed).norm() < vector_read_back);
    }
}

TEST_CASE("the twist opens off, stands for nothing while no rates are told, and answers its own switch", "[manipulator][drawing]")
{
    column_stage headless;
    CHECK_FALSE(headless.shown.twist_shown());

    REQUIRE(headless.shown.set_jacobian_columns(2u).has_value());
    headless.shown.set_twist_shown(true);
    drawn_as(headless, false, false);

    REQUIRE(headless.shown.set_joint_rates(told_rates).has_value());
    headless.shown.set_twist_shown(false);
    drawn_as(headless, false, false);

    headless.shown.set_twist_shown(true);
    CHECK(headless.shown.twist_shown());
    drawn_as(headless, true, true);
}

TEST_CASE("the twist is hidden where the shown Jacobian is refused or its width or the tool pose cannot place it", "[manipulator][drawing]")
{
    const Eigen::Matrix<double, 6, 1> third = twist_of(Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0));
    const jacobian wider                    = of_columns({two_columns(1.0).col(0), two_columns(1.0).col(1), third});
    hidden_once(published_columns(wider, wider, elsewhere), jacobian_frame::space);
    hidden_once(published_columns(two_columns(1.0), two_columns(1.0), praxis::unexpected(refusal::unsupported_input)), jacobian_frame::space);
    hidden_once(published_columns(two_columns(1.0), praxis::unexpected(refusal::not_implemented), elsewhere), jacobian_frame::body);

    arm_snapshot unturned     = published_columns(two_columns(1.0), two_columns(1.0), elsewhere);
    unturned.tool_orientation = praxis::unexpected(refusal::unsupported_input);
    hidden_once(unturned, jacobian_frame::body);
}

TEST_CASE("a part's switch withholds that part of the twist and leaves the other", "[manipulator][drawing]")
{
    column_stage headless;
    tell(headless, published_columns(two_columns(1.0), two_columns(1.0), elsewhere));

    headless.shown.set_column_part_shown(jacobian_block::angular, false);
    drawn_as(headless, false, true);
    headless.shown.set_column_part_shown(jacobian_block::angular, true);
    drawn_as(headless, true, true);
    headless.shown.set_column_part_shown(jacobian_block::linear, false);
    drawn_as(headless, true, false);
    headless.shown.set_column_part_shown(jacobian_block::linear, true);
    drawn_as(headless, true, true);
}

TEST_CASE("the twist and the columns each answer their own switch, and the twist stands weighted or not", "[manipulator][drawing]")
{
    column_stage headless;
    tell(headless, published_columns(two_columns(1.0), two_columns(1.0), elsewhere));

    headless.shown.set_jacobian_columns_shown(false);
    drawn_as(headless, true, true);
    CHECK_FALSE(drawn(headless.arrow(0u, jacobian_block::angular)));

    headless.shown.set_jacobian_columns_shown(true);
    headless.shown.set_twist_shown(false);
    drawn_as(headless, false, false);
    for(std::size_t column = 0; column < 2u; ++column)
        for(const jacobian_block part : both_parts)
            CHECK(drawn(headless.arrow(column, part)));

    headless.shown.set_twist_shown(true);
    headless.shown.set_columns_weighted(true);
    drawn_as(headless, true, true);
}

TEST_CASE("each twist arrow wears its part's twist tone and hangs in a subtree apart from the columns", "[manipulator][drawing]")
{
    column_stage headless;
    tell(headless, published_columns(two_columns(1.0), two_columns(1.0), elsewhere));

    for(const jacobian_block part : both_parts)
    {
        threepp::Object3D *const twist = twist_arrow(headless, part);
        REQUIRE(twist != nullptr);
        CHECK(arrow_tone(twist) == twist_tone(part));
        CHECK(twist->parent->parent != headless.arrow(0u, part)->parent->parent);
        CHECK(twist->parent->parent->parent == headless.arrow(0u, part)->parent->parent->parent);
    }
}

TEST_CASE("a tool hung from the flange, turned and moved, leaves the twist the same two vectors in both frames", "[manipulator][drawing]")
{
    const arm_snapshot seen = hung_from(posed(turned_tool(), elsewhere), tool_hung);
    column_stage headless;
    tell(headless, seen);

    const Eigen::Matrix<double, 6, 1> twist = at_tool_point(times_rates(*seen.space_jacobian), 0, *seen.tool_position);
    const Eigen::Vector3d analytic[]        = {twist.head<3>() * angular_column_scale, twist.tail<3>() * linear_column_scale};
    std::vector<Eigen::Vector3d> in_space;
    for(const jacobian_block part : both_parts)
        in_space.push_back(drawn_vector(twist_arrow(headless, part)));

    headless.shown.set_jacobian_frame(jacobian_frame::body);
    headless.draw();
    for(const jacobian_block part : both_parts)
    {
        const std::size_t at = static_cast<std::size_t>(part);
        CHECK((arrow_at(twist_arrow(headless, part)) - *seen.tool_position).norm() < placed_back);
        CHECK((drawn_vector(twist_arrow(headless, part)) - in_space[at]).norm() < vector_read_back);
        CHECK((in_space[at] - analytic[at]).norm() < vector_read_back);
        CHECK((drawn_vector(twist_arrow(headless, part)) - analytic[at]).norm() < vector_read_back);
    }
}

TEST_CASE("under a tool hung from the flange each body column stands at the tool as the velocity its joint gives the tool point, and the weighted columns sum to the twist",
          "[manipulator][drawing]")
{
    const arm_snapshot seen = hung_from(posed(turned_tool(), elsewhere), tool_hung);
    column_stage headless;
    tell(headless, seen, jacobian_frame::body);
    headless.shown.set_columns_weighted(true);
    headless.draw();

    for(const jacobian_block part : both_parts)
    {
        const Eigen::Index rows = part == jacobian_block::angular ? 0 : 3;
        const double scale      = part == jacobian_block::angular ? angular_column_scale : linear_column_scale;
        Eigen::Vector3d summed  = Eigen::Vector3d::Zero();
        for(std::size_t column = 0; column < 2u; ++column)
        {
            const Eigen::Matrix<double, 6, 1> given = at_tool_point(*seen.space_jacobian, static_cast<Eigen::Index>(column), *seen.tool_position);
            CHECK((arrow_at(headless.arrow(column, part)) - *seen.tool_position).norm() < placed_back);
            CHECK((drawn_vector(headless.arrow(column, part)) - told_rates[column] * given.segment<3>(rows) * scale).norm() < vector_read_back);
            summed += drawn_vector(headless.arrow(column, part));
        }
        CHECK((drawn_vector(twist_arrow(headless, part)) - summed).norm() < vector_read_back);
    }
}
