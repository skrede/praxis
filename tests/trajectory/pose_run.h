#ifndef HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_POSE_RUN_H
#define HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_POSE_RUN_H

#include "praxis/trajectory/capabilities.h"
#include "praxis/trajectory/pose_trajectory.h"

#include "praxis/trajectory/baseline/pose_trajectory.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <span>
#include <memory>
#include <numbers>

namespace praxis::fixture {

using namespace trajectory;

inline transform assembled(const rotation &r, const Eigen::Vector3d &p)
{
    transform tf         = transform::Identity();
    tf.block<3, 3>(0, 0) = r;
    tf.block<3, 1>(0, 3) = p;

    return tf;
}

inline transform turned_about(const Eigen::Vector3d &axis, double radians, const Eigen::Vector3d &p)
{
    return assembled(Eigen::AngleAxisd(radians, axis.normalized()).toRotationMatrix(), p);
}

inline transform seed_frame()
{
    return assembled(rotation::Identity(), Eigen::Vector3d{1.0, 0.0, 0.25});
}

// A quarter turn about the world z with a translation, so the rotation is about no axis through
// either origin and the exponential coordinates of the motion curve in the path parameter.
inline transform waypoint()
{
    return turned_about(Eigen::Vector3d::UnitZ(), std::numbers::pi_v<double> / 2.0, Eigen::Vector3d{0.0, 1.5, 0.75});
}

using pose_run = decltype(pose_trajectory_ops::decoupled_pose_waypoints);

// The run a factory builds when handed the reference screw, frame, path and time-scaling operations.
inline expected<std::unique_ptr<pose_trajectory_generator>, refusal> referenced(pose_run factory, std::span<const transform> poses, const transform &seed, double max_linear_speed,
                                                                                double max_angular_speed)
{
    const rigid_motion::capabilities rigid = rigid_motion::baseline();
    const trajectory::capabilities shapes  = trajectory::baseline();

    return factory(rigid.screw, rigid.frame, shapes.path, shapes.time_scaling, poses, seed, max_linear_speed, max_angular_speed);
}

inline pose_sample at(const pose_trajectory_generator &motion, double t)
{
    auto sampled = motion.sample(t);
    REQUIRE(sampled);

    return *sampled;
}

}

#endif
