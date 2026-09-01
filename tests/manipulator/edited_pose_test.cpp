#include "praxis/manipulator/edited_pose.h"

#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/axis_order.h"
#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

using namespace praxis;
using namespace praxis::manipulator;

namespace {

const rigid_motion::frame_ops reference = rigid_motion::baseline().frame;

// No two of them equal, none a right angle or a straight one, and the first inside the half turn an
// extraction confines a first angle to, so a triple extracted back is the triple that went in and a
// conversion taken the wrong way lands where none of them is.
const Eigen::Vector3d chosen_euler_degrees{37.0, 52.0, -19.0};
const Eigen::Vector3d chosen_position{0.25, -0.4, 0.7};

// The pose holds its components in single precision, so a degree read back is the degree written to
// within one float step of it, which is 4e-6 at the widest of these angles.
constexpr double float_step = 1.0e-4;

arm_snapshot published_at(const expected<Eigen::Vector3d, refusal> &position, const expected<rotation, refusal> &orientation)
{
    const transform identity = transform::Identity();

    return arm_snapshot{joint_vector(),
                        joint_limits{},
                        identity,
                        identity,
                        identity,
                        position,
                        position,
                        orientation,
                        orientation,
                        recording_parameters{},
                        1.0,
                        false,
                        scheduler::task_counters{},
                        {},
                        unexpected(refusal::not_implemented),
                        unexpected(refusal::not_implemented),
                        jacobian_manipulability{unexpected(refusal::not_implemented), unexpected(refusal::not_implemented)},
                        jacobian_manipulability{unexpected(refusal::not_implemented), unexpected(refusal::not_implemented)},
                        {},
                        {},
                        nullptr,
                        nullptr,
                        {},
                        {}};
}

}

TEST_CASE("a pose carrying no offset and no rotation composes the identity transform", "[manipulator][controls]")
{
    const edited_pose edited;

    REQUIRE(edited.order == axis_order::zyx);
    CHECK(pose_matrix(edited, reference).isApprox(transform::Identity()));
}

TEST_CASE("one position and one angle triple compose two transforms under two axis orders", "[manipulator][controls]")
{
    edited_pose first;
    first.position      = chosen_position.cast<float>();
    first.euler_degrees = chosen_euler_degrees.cast<float>();

    edited_pose second = first;
    first.order        = axis_order::zyx;
    second.order       = axis_order::xyz;

    CHECK_FALSE(pose_matrix(first, reference).isApprox(pose_matrix(second, reference)));
}

TEST_CASE("seeding from a published tool pose takes the position as it stands and the orientation in degrees", "[manipulator][controls]")
{
    edited_pose edited;
    const rotation orientation = reference.rotation_matrix_from_euler(chosen_euler_degrees * radians_per_degree, edited.order);

    REQUIRE(seed_from(edited, published_at(chosen_position, orientation), reference));

    for(Eigen::Index axis = 0; axis < 3; ++axis)
    {
        CHECK(static_cast<double>(edited.position[axis]) == Catch::Approx(chosen_position[axis]).margin(float_step));
        CHECK(static_cast<double>(edited.euler_degrees[axis]) == Catch::Approx(chosen_euler_degrees[axis]).margin(float_step));
    }
}

TEST_CASE("seeding from a publication carrying no tool pose leaves the pose as it stands", "[manipulator][controls]")
{
    edited_pose edited;
    edited.position      = chosen_position.cast<float>();
    edited.euler_degrees = chosen_euler_degrees.cast<float>();

    const arm_snapshot no_position    = published_at(unexpected(refusal::no_solution), rotation::Identity());
    const arm_snapshot no_orientation = published_at(Eigen::Vector3d::Zero(), unexpected(refusal::no_solution));

    CHECK_FALSE(seed_from(edited, no_position, reference));
    CHECK_FALSE(seed_from(edited, no_orientation, reference));
    CHECK(edited.position.isApprox(chosen_position.cast<float>()));
    CHECK(edited.euler_degrees.isApprox(chosen_euler_degrees.cast<float>()));
}
