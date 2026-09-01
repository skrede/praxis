#include "answered.h"

#include "praxis/trajectory/baseline/path.h"

#include "praxis/rigid_motion/baseline/frame.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <numbers>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::trajectory;

namespace {

transform assembled(const rotation &r, const Eigen::Vector3d &p)
{
    transform tf         = transform::Identity();
    tf.block<3, 3>(0, 0) = r;
    tf.block<3, 1>(0, 3) = p;
    return tf;
}

// A quarter turn about the world z together with a translation, so the rotation is not about an axis
// through either origin: that is what separates a screw path from a decoupled one.
transform start_frame()
{
    return assembled(rotation::Identity(), Eigen::Vector3d{1.0, 0.0, 0.25});
}

transform end_frame()
{
    return assembled(Eigen::AngleAxisd(std::numbers::pi_v<double> / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix(), Eigen::Vector3d{0.0, 1.0, 0.75});
}

Eigen::Vector3d origin_of(const transform &tf)
{
    return tf.block<3, 1>(0, 3);
}

bool is_a_rigid_motion(const transform &tf)
{
    const rotation r = rigid_motion::rotation_matrix_from_transform(tf);

    return is_approx_equal(rotation(r.transpose() * r), rotation::Identity()) && is_approx_equal(r.determinant(), 1.0) &&
            is_approx_equal((tf.row(3) - Eigen::RowVector4d::UnitW()).cwiseAbs().maxCoeff(), 0.0);
}

configuration pair_of(double first, double second)
{
    configuration q(2);
    q << first, second;

    return q;
}

}

TEST_CASE("a_configuration_space_straight_line_holds_its_endpoints_and_interpolates_between_them")
{
    const configuration start = pair_of(0.25, -1.0);
    const configuration end   = pair_of(-0.75, 2.0);

    CHECK(is_approx_equal(answered(joint_straight_line(start, end, 0.0)), start));
    CHECK(is_approx_equal(answered(joint_straight_line(start, end, 1.0)), end));
    CHECK(is_approx_equal(answered(joint_straight_line(start, end, 0.5)), pair_of(-0.25, 0.5)));
    CHECK(is_approx_equal(answered(joint_straight_line(start, end, 0.25)), pair_of(0.0, -0.25)));
}

TEST_CASE("a_straight_line_between_configurations_of_different_sizes_is_refused_rather_than_held_at_the_start")
{
    const configuration start = pair_of(0.25, -1.0);
    const configuration end   = configuration::Constant(3, 1.0);

    const auto refused = joint_straight_line(start, end, 1.0);

    REQUIRE(!refused);
    CHECK(refused.error() == refusal::unsupported_input);
}

// A matrix outside SE(3) has no matrix logarithm, so a path taken between two of them would report a
// pose no caller could tell from one the mathematics actually produced.
TEST_CASE("a_task_space_path_between_endpoints_that_are_not_rigid_motions_is_refused")
{
    const transform skewed = assembled(2.0 * rotation::Identity(), Eigen::Vector3d{1.0, 0.0, 0.25});

    const auto refused_screw     = screw(skewed, end_frame(), 0.5);
    const auto refused_decoupled = decoupled(start_frame(), skewed, 0.5);

    REQUIRE(!refused_screw);
    CHECK(refused_screw.error() == refusal::degenerate);
    REQUIRE(!refused_decoupled);
    CHECK(refused_decoupled.error() == refusal::degenerate);
}

// Both paths premultiply by the start frame, so a bottom row that is not affine survives into the
// pose they report. The relative motion between the two endpoints is a rigid motion here -- taking
// the inverse of the start rebuilds its bottom row -- so the logarithms each path takes have an
// answer for it and refuse nothing; only reading each endpoint as a member of SE(3) catches it.
TEST_CASE("an_endpoint_whose_bottom_row_is_not_affine_is_refused_even_though_the_motion_between_the_two_is_rigid")
{
    transform unscaled = start_frame();
    unscaled.row(3)    = Eigen::Vector4d{0.0, 0.0, 0.0, 2.0}.transpose();

    const auto refused_screw     = screw(unscaled, end_frame(), 0.5);
    const auto refused_decoupled = decoupled(unscaled, end_frame(), 0.5);

    REQUIRE(is_a_rigid_motion(rigid_motion::inverse(unscaled) * end_frame()));
    REQUIRE(!refused_screw);
    CHECK(refused_screw.error() == refusal::degenerate);
    REQUIRE(!refused_decoupled);
    CHECK(refused_decoupled.error() == refusal::degenerate);
}

TEST_CASE("both_task_space_paths_hold_their_endpoints")
{
    CHECK(is_approx_equal(answered(screw(start_frame(), end_frame(), 0.0)), start_frame(), 1.0e-9));
    CHECK(is_approx_equal(answered(screw(start_frame(), end_frame(), 1.0)), end_frame(), 1.0e-9));
    CHECK(is_approx_equal(answered(decoupled(start_frame(), end_frame(), 0.0)), start_frame(), 1.0e-9));
    CHECK(is_approx_equal(answered(decoupled(start_frame(), end_frame(), 1.0)), end_frame(), 1.0e-9));
}

TEST_CASE("both_task_space_paths_are_rigid_motions_throughout")
{
    for(int step = 0; step <= 10; ++step)
    {
        const double s = static_cast<double>(step) / 10.0;
        CHECK(is_a_rigid_motion(answered(screw(start_frame(), end_frame(), s))));
        CHECK(is_a_rigid_motion(answered(decoupled(start_frame(), end_frame(), s))));
    }
}

TEST_CASE("the_decoupled_path_carries_its_origin_along_the_straight_line")
{
    const Eigen::Vector3d from = origin_of(start_frame());
    const Eigen::Vector3d to   = origin_of(end_frame());

    for(int step = 0; step <= 10; ++step)
    {
        const double s = static_cast<double>(step) / 10.0;
        CHECK(origin_of(answered(decoupled(start_frame(), end_frame(), s))).isApprox(from + s * (to - from), 1.0e-9));
    }
}

// The property the two paths differ by, and the reason both slots exist: a constant screw axis whose
// line misses the frame origins drags that origin off the straight line between the endpoints.
TEST_CASE("the_screw_path_leaves_the_straight_line_its_endpoints_span")
{
    const Eigen::Vector3d from   = origin_of(start_frame());
    const Eigen::Vector3d to     = origin_of(end_frame());
    const Eigen::Vector3d midway = origin_of(answered(screw(start_frame(), end_frame(), 0.5)));

    CHECK((midway - 0.5 * (from + to)).norm() > 0.1);
}

// A screw path is the one-parameter subgroup through the relative motion, so the motion between two
// path parameters depends on their difference and not on where in the path they sit.
TEST_CASE("the_screw_path_advances_by_the_same_relative_motion_everywhere")
{
    const transform first  = answered(screw(start_frame(), end_frame(), 0.25));
    const transform second = answered(screw(start_frame(), end_frame(), 0.50));
    const transform third  = answered(screw(start_frame(), end_frame(), 0.75));
    const transform early  = first.inverse() * second;
    const transform late   = second.inverse() * third;

    CHECK(is_approx_equal(early, late, 1.0e-9));
}

TEST_CASE("a_task_space_path_between_a_frame_and_itself_stands_still")
{
    CHECK(is_approx_equal(answered(screw(start_frame(), start_frame(), 0.6)), start_frame(), 1.0e-9));
    CHECK(is_approx_equal(answered(decoupled(start_frame(), start_frame(), 0.6)), start_frame(), 1.0e-9));
}
