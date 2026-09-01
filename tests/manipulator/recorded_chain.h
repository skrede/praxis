#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_RECORDED_CHAIN_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_RECORDED_CHAIN_H

#include "praxis/manipulator/screw_chain.h"

#include "praxis/rigid_motion/baseline/frame.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/LU>

#include <meios/urdf/load.h>
#include <meios/model.h>

#include <vector>
#include <cstddef>
#include <numbers>
#include <optional>
#include <filesystem>

// The descriptions are deployed beside the demonstration executable, so a configure without the
// demonstration leaves nothing to parse; that is a valid configuration and skips.
#ifndef PRAXIS_DEMO_RESOURCE_DIR
    #define PRAXIS_DEMO_RESOURCE_DIR ""
#endif

namespace praxis::fixture {

using namespace manipulator;

inline std::filesystem::path description_path(const std::filesystem::path &root)
{
    return root / "kuka_kr6_support/urdf/kr6r900sixx.xacro";
}

inline std::optional<meios::model<>> deployed_description()
{
    const std::filesystem::path root{PRAXIS_DEMO_RESOURCE_DIR};
    if(root.empty() || !std::filesystem::exists(description_path(root)))
        return std::nullopt;

    meios::load_options options;
    options.package_roots.push_back(root);
    options.eval       = meios::eval_policy::fail;
    options.on_missing = meios::missing_asset::fail;

    auto loaded = meios::load(description_path(root), options);
    REQUIRE(loaded.has_value());

    return loaded->robot;
}

inline bool is_rigid_motion(const transform &pose)
{
    const rotation r = rigid_motion::rotation_matrix_from_transform(pose);

    return is_approx_equal(rotation(r.transpose() * r), rotation::Identity()) && is_approx_equal(r.determinant(), 1.0) &&
            is_approx_equal((pose.row(3) - Eigen::RowVector4d::UnitW()).cwiseAbs().maxCoeff(), 0.0);
}

inline bool is_unit_screw(const screw_axis &axis)
{
    const double angular = axis.head<3>().norm();
    const double linear  = axis.tail<3>().norm();

    return is_approx_equal(angular, 1.0, 1.0e-9) || (is_approx_equal(angular, 0.0, 1.0e-9) && is_approx_equal(linear, 1.0, 1.0e-9));
}

inline constexpr double in_radians(double degrees)
{
    return degrees * std::numbers::pi / 180.0;
}

// Everything below is transcribed from the deployed description's own joint origins, axes and limit
// elements. Each screw is written in the root link's frame as the joint axis rotated into that
// frame together with the moment of that axis about the frame's origin.
inline constexpr std::size_t recorded_joint_count = 6u;

inline transform recorded_home()
{
    transform pose = transform::Identity();
    pose.block<3, 3>(0, 0) << 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0;
    pose.block<3, 1>(0, 3) << 0.980, 0.0, 0.435;

    return pose;
}

inline std::vector<screw_axis> recorded_screws()
{
    std::vector<screw_axis> screws(recorded_joint_count, screw_axis::Zero());
    screws[0] << 0.0, 0.0, -1.0, 0.000, 0.000, 0.000;
    screws[1] << 0.0, 1.0, 0.0, -0.400, 0.000, 0.025;
    screws[2] << 0.0, 1.0, 0.0, -0.400, 0.000, 0.480;
    screws[3] << -1.0, 0.0, 0.0, 0.000, -0.435, 0.000;
    screws[4] << 0.0, 1.0, 0.0, -0.435, 0.000, 0.900;
    screws[5] << -1.0, 0.0, 0.0, 0.000, -0.435, 0.000;

    return screws;
}

inline joint_vector recorded_lower_bounds()
{
    joint_vector bounds(recorded_joint_count);
    bounds << -in_radians(170.0), -in_radians(190.0), -in_radians(120.0), -in_radians(185.0), -in_radians(120.0), -in_radians(350.0);

    return bounds;
}

inline joint_vector recorded_upper_bounds()
{
    joint_vector bounds(recorded_joint_count);
    bounds << in_radians(170.0), in_radians(45.0), in_radians(156.0), in_radians(185.0), in_radians(120.0), in_radians(350.0);

    return bounds;
}

inline joint_vector recorded_velocity_bounds()
{
    joint_vector bounds(recorded_joint_count);
    bounds << in_radians(360.0), in_radians(300.0), in_radians(360.0), in_radians(381.0), in_radians(388.0), in_radians(615.0);

    return bounds;
}

// The description declares no acceleration, so the derivation takes its own ratio of the declared
// velocity.
inline constexpr double recorded_acceleration_ratio = 0.5;

inline joint_vector recorded_configuration(double first, double second, double third, double fourth, double fifth, double sixth)
{
    joint_vector q(recorded_joint_count);
    q << first, second, third, fourth, fifth, sixth;

    return q;
}

inline screw_chain recorded_arm()
{
    joint_limits bounds{};
    bounds.lower_position = recorded_lower_bounds();
    bounds.upper_position = recorded_upper_bounds();
    bounds.velocity       = recorded_velocity_bounds();
    bounds.acceleration   = recorded_velocity_bounds() * recorded_acceleration_ratio;

    return screw_chain(recorded_home(), recorded_screws(), bounds);
}

// Only the second joint moves, by a quarter turn about the y axis it carries at the zero
// configuration, which stands the arm up: the tool origin lands on the sum of the link offsets
// above the base and the first joint's own offset behind it.
inline joint_vector recorded_raised_configuration()
{
    joint_vector q = joint_vector::Zero(recorded_joint_count);
    q[1]           = -std::numbers::pi / 2.0;

    return q;
}

inline transform recorded_raised_pose()
{
    transform pose = transform::Identity();
    pose.block<3, 1>(0, 3) << -0.010, 0.0, 1.355;

    return pose;
}

}

#endif
