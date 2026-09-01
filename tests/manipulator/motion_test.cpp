#include "praxis/manipulator/motion.h"

#include <catch2/catch_test_macros.hpp>

#include <memory>

using namespace praxis;
using namespace praxis::manipulator;

TEST_CASE("the_motion_resolution_slots_default_to_refusing_rather_than_to_the_seed_configuration")
{
    motion_ops ops{};
    const kinematics solver{};
    const joint_vector j0 = joint_vector::Constant(3, 0.25);

    REQUIRE(ops.task_space_pose != nullptr);
    REQUIRE(ops.task_space_screw != nullptr);
    REQUIRE(ops.tool_frame_displace != nullptr);

    const expected<joint_vector, refusal> to_pose = ops.task_space_pose(solver, transform::Identity(), j0);
    const expected<joint_vector, refusal> to_screw =
            ops.task_space_screw(rigid_motion::screw_ops{}, solver, transform::Identity(), Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(), 1.5, 0.5, j0);
    const expected<joint_vector, refusal> jogged = ops.tool_frame_displace(solver, transform::Identity(), Eigen::Vector3d::UnitX(), rotation::Identity(), j0);

    REQUIRE_FALSE(to_pose.has_value());
    REQUIRE_FALSE(to_screw.has_value());
    REQUIRE_FALSE(jogged.has_value());
    CHECK(to_pose.error() == refusal::not_implemented);
    CHECK(to_screw.error() == refusal::not_implemented);
    CHECK(jogged.error() == refusal::not_implemented);
}
