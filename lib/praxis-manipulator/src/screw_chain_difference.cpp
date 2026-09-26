#include "praxis/manipulator/screw_chain_difference.h"

#include "praxis/evaluation/comparators.h"

#include "praxis/rigid_motion/types.h"

#include <Eigen/SVD>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>

namespace praxis::manipulator {

namespace {

// How far a supplied home block may stand from being a rotation and still have a turn read off the
// rotation nearest it.
constexpr double home_rigidity_bound = 1.0e-1;

// The boundary the drawing reads the same screw at, so a row drawn as translating is a row compared
// as translating.
bool translates(const screw_axis &screw)
{
    return screw.head<3>().norm() <= angular_epsilon;
}

// Taken from the cross product beside the dot product rather than from an arc cosine of the dot
// product alone, which loses its precision at both ends of its range and answers exactly a half turn
// for a pair that is exactly opposed.
double angle_between(const Eigen::Vector3d &first, const Eigen::Vector3d &second)
{
    return std::atan2(first.cross(second).norm(), first.dot(second));
}

// The greater of the two defects rather than the supplied side's alone: a non-unit axis on the
// derived side is this library's own defect and must not pass unseen.
double length_defect(const Eigen::Vector3d &first, const Eigen::Vector3d &second)
{
    return std::max(std::fabs(first.norm() - 1.0), std::fabs(second.norm() - 1.0));
}

// A reflection is exactly orthonormal and a shear leaves the determinant where it was, so each term
// catches a class the other misses.
double rigidity_defect(const Eigen::Matrix3d &block)
{
    const Eigen::Matrix3d gram(block.transpose() * block);

    return std::max((gram - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(), std::fabs(block.determinant() - 1.0));
}

// The rotation nearest a block in the Frobenius sense, with the last singular direction flipped
// where the decomposition would otherwise answer a reflection.
Eigen::Matrix3d nearest_rotation(const Eigen::Matrix3d &block)
{
    const Eigen::JacobiSVD<Eigen::Matrix3d> taken(block, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix3d turned(taken.matrixU() * taken.matrixV().transpose());

    Eigen::Matrix3d handed = Eigen::Matrix3d::Identity();
    handed(2, 2)           = turned.determinant() < 0.0 ? -1.0 : 1.0;

    return taken.matrixU() * handed * taken.matrixV().transpose();
}

// A linear half of no length names no direction either, so the angle between two of them is left
// unanswered rather than taken from a vector that points nowhere.
chain_joint_difference both_translating(const screw_axis &held, const screw_axis &against)
{
    const Eigen::Vector3d first(held.tail<3>());
    const Eigen::Vector3d second(against.tail<3>());
    const bool directed = first.norm() > 0.0 && second.norm() > 0.0;

    return chain_joint_difference{chain_joint_reading::measured, directed ? std::optional<double>(angle_between(first, second)) : std::optional<double>(), length_defect(first, second),
                                  std::optional<double>()};
}

chain_joint_difference both_turning(const screw_axis &held, const screw_axis &against)
{
    const Eigen::Vector3d first(held.head<3>());
    const Eigen::Vector3d second(against.head<3>());
    const double apart = (held.tail<3>() / first.norm() - against.tail<3>() / second.norm()).norm();

    return chain_joint_difference{chain_joint_reading::measured, angle_between(first.normalized(), second.normalized()), length_defect(first, second), apart};
}

chain_joint_difference compared(const screw_axis &held, const screw_axis &against)
{
    if(translates(held) != translates(against))
        return chain_joint_difference{chain_joint_reading::kinds_differed, std::optional<double>(), std::optional<double>(), std::optional<double>()};

    return translates(held) ? both_translating(held, against) : both_turning(held, against);
}

chain_home_difference home_compared(const screw_chain &derived, const transform &supplied_home)
{
    const Eigen::Matrix3d held(supplied_home.block<3, 3>(0, 0));
    const Eigen::Matrix3d described(derived.home.block<3, 3>(0, 0));
    const double rigidity = std::max(rigidity_defect(held), rigidity_defect(described));
    const double moved    = (supplied_home.block<3, 1>(0, 3) - derived.home.block<3, 1>(0, 3)).norm();

    if(rigidity > home_rigidity_bound)
        return chain_home_difference{std::numeric_limits<double>::infinity(), moved, rigidity};

    return chain_home_difference{evaluation::geodesic_residual(nearest_rotation(held), described).magnitude, moved, rigidity};
}

}

screw_chain_difference supplied_chain_difference(const screw_chain &derived, const transform &supplied_home, std::span<const supplied_screw> supplied)
{
    const chain_joint_difference unsupplied{chain_joint_reading::not_supplied, std::optional<double>(), std::optional<double>(), std::optional<double>()};

    std::vector<chain_joint_difference> joints;
    joints.reserve(derived.joint_count());
    for(std::size_t joint = 0u; joint < derived.joint_count(); ++joint)
    {
        const bool held = joint < supplied.size() && supplied[joint].has_value();

        joints.push_back(held ? compared(*supplied[joint], derived.space_screws[joint]) : unsupplied);
    }

    return screw_chain_difference{home_compared(derived, supplied_home), std::move(joints), supplied.size()};
}

}
