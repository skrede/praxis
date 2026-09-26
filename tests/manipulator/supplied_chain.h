#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_SUPPLIED_CHAIN_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_SUPPLIED_CHAIN_H

#include "praxis/manipulator/screw_chain.h"
#include "praxis/manipulator/screw_chain_difference.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <span>
#include <cmath>
#include <vector>
#include <cstddef>
#include <numbers>

namespace praxis::fixture {

using namespace manipulator;

inline constexpr double exactly = 1.0e-12;
inline constexpr double nearly  = 1.0e-9;

inline constexpr double a_quarter_turn = std::numbers::pi_v<double> / 2.0;
inline constexpr double a_half_turn    = std::numbers::pi_v<double>;

// The angle the home pose below stands at, which is the derived side's own number and therefore the
// one no reading of a supplied block may report.
inline constexpr double a_home_turn = 0.4;

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

    placed.block<3, 3>(0, 0) = Eigen::AngleAxisd(a_home_turn, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()).toRotationMatrix();
    placed.block<3, 1>(0, 3) = Eigen::Vector3d(0.1, -0.2, 0.7);

    return placed;
}

inline screw_chain a_chain()
{
    return screw_chain(a_home_pose(), described_screws(), joint_limits{});
}

// A chain every joint of which somebody supplied, which is what a caller handing a bare list of
// screws means by it.
inline std::vector<supplied_screw> all_supplied(std::vector<screw_axis> screws)
{
    return std::vector<supplied_screw>(screws.begin(), screws.end());
}

inline transform home_pose_whose_rotation_is(const rotation &block)
{
    transform placed         = a_home_pose();
    placed.block<3, 3>(0, 0) = block;

    return placed;
}

// Neither block stands near a rotation: every direction of the first has been lost and all but one
// of the second's.
inline rotation no_frame_at_all()
{
    return rotation::Zero();
}

inline rotation one_direction_only()
{
    rotation single = rotation::Zero();
    single.col(0)   = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();

    return single;
}

// A unit shear leaves the determinant where it was and stands exactly its own off-diagonal entry
// from orthonormality, so the block below carries the rigidity defect it was asked for rather than a
// rounded neighbour of it.
inline rotation block_whose_rigidity_defect_is(double defect)
{
    rotation sheared = rotation::Identity();
    sheared(0, 1)    = defect;

    return sheared;
}

// The greatest rigidity defect a supplied home block may carry and still be answered a turn, found
// by halving rather than written down, so a case built on it measures whatever the comparison admits
// rather than a number copied beside it.
inline double greatest_admitted_rigidity_defect()
{
    const screw_chain derived = a_chain();

    double answered = 0.0;
    double refused  = 1.0;
    while(std::nextafter(answered, refused) != refused)
    {
        const double between  = answered + (refused - answered) / 2.0;
        const transform tried = home_pose_whose_rotation_is(block_whose_rigidity_defect_is(between));

        if(std::isfinite(supplied_chain_difference(derived, tried, std::span<const supplied_screw>()).home.turned_radians))
            answered = between;
        else
            refused = between;
    }

    return answered;
}

inline std::vector<screw_axis> with_one_changed(std::vector<screw_axis> screws, std::size_t joint, const screw_axis &instead)
{
    screws[joint] = instead;

    return screws;
}

}

#endif
