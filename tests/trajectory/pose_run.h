#ifndef HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_POSE_RUN_H
#define HPP_GUARD_PRAXIS_TESTS_TRAJECTORY_POSE_RUN_H

#include "praxis/trajectory/baseline/pose_trajectory.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

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

inline pose_sample at(const pose_trajectory_generator &motion, double t)
{
    auto sampled = motion.sample(t);
    REQUIRE(sampled);

    return *sampled;
}

}

#endif
