#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_SUPPLIED_CHAIN_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_SUPPLIED_CHAIN_H

#include "praxis/manipulator/screw_chain.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <cstddef>
#include <numbers>

namespace praxis::fixture {

using namespace manipulator;

inline constexpr double exactly = 1.0e-12;
inline constexpr double nearly  = 1.0e-9;

inline constexpr double a_quarter_turn = std::numbers::pi_v<double> / 2.0;
inline constexpr double a_half_turn    = std::numbers::pi_v<double>;

inline constexpr std::size_t chain_joints      = 4u;
inline constexpr std::size_t turning_joint     = 1u;
inline constexpr std::size_t translating_joint = 3u;

inline screw_axis axis_of(double wx, double wy, double wz, double vx, double vy, double vz)
{
    screw_axis written;

    written << wx, wy, wz, vx, vy, vz;

    return written;
}

// Three joints turn, about three different directions and each with a nonzero moment, and the last
// one translates, so one chain reaches every branch the comparison has.
inline std::vector<screw_axis> described_screws()
{
    return {axis_of(0.0, 0.0, 1.0, 0.0, -0.3, 0.0), axis_of(0.0, 1.0, 0.0, 0.2, 0.0, -0.4), axis_of(1.0, 0.0, 0.0, 0.0, 0.5, 0.1), axis_of(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)};
}

inline transform a_home_pose()
{
    transform placed = transform::Identity();

    placed.block<3, 3>(0, 0) = Eigen::AngleAxisd(0.4, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()).toRotationMatrix();
    placed.block<3, 1>(0, 3) = Eigen::Vector3d(0.1, -0.2, 0.7);

    return placed;
}

inline screw_chain a_chain()
{
    return screw_chain(a_home_pose(), described_screws(), joint_limits{});
}

inline std::vector<screw_axis> with_one_changed(std::vector<screw_axis> screws, std::size_t joint, const screw_axis &instead)
{
    screws[joint] = instead;

    return screws;
}

}

#endif
