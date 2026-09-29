#include "robot/chain_placement.h"

#include "praxis/rigid_motion/types.h"

#include <Eigen/LU>
#include <Eigen/Core>

#include <span>
#include <cmath>
#include <limits>
#include <vector>
#include <cstddef>
#include <algorithm>

namespace praxis::manipulator {

namespace {

// The rigidity defect up to which an exponential is still folded into a chain's end.
constexpr double exponential_rigidity_bound = 1.0e-2;

// A defect that is not a number compares false, so it is refused.
bool is_rigid_motion(const transform &placed)
{
    return rigidity_defect(placed) <= exponential_rigidity_bound;
}

// The point of the axis a carried screw names that stands nearest the point before it. A screw with
// an angular part passes through the angular direction crossed into the linear part, which is one
// point of that axis and stands in for every other: the projection answers the same point whichever
// of them it is measured from. A screw with no angular part names no axis at all.
Eigen::Vector3d next_origin(const Eigen::Vector3d &before, const twist &carried_screw)
{
    const Eigen::Vector3d angular = carried_screw.head<3>();
    if(angular.norm() <= angular_epsilon)
        return before;

    const Eigen::Vector3d along   = angular.normalized();
    const Eigen::Vector3d through = along.cross(carried_screw.tail<3>() / angular.norm());

    return through + along * (before - through).dot(along);
}

}

expected<std::vector<Eigen::Vector3d>, refusal> fold_joint_origins(const transform &home, std::span<const screw_axis> space_screws, const joint_vector &theta,
                                                                   const rigid_motion::screw_ops &screw)
{
    std::vector<Eigen::Vector3d> points;
    points.reserve(space_screws.size() + 2u);
    points.emplace_back(Eigen::Vector3d::Zero());

    transform carried = transform::Identity();
    for(std::size_t joint = 0; joint < space_screws.size(); ++joint)
    {
        const expected<twist, refusal> moved = screw.adjoint_map(space_screws[joint], carried);
        if(!moved)
            return unexpected(moved.error());

        points.push_back(next_origin(points.back(), *moved));

        const auto at     = static_cast<Eigen::Index>(joint);
        const double turn = at < theta.size() ? theta[at] : 0.0;
        carried           = transform(carried * screw.matrix_exponential_screw(space_screws[joint], turn));
    }

    const transform reached = carried * home;
    points.emplace_back(reached.block<3, 1>(0, 3));

    return points;
}

double rigidity_defect(const transform &placed)
{
    if(!placed.allFinite())
        return std::numeric_limits<double>::quiet_NaN();

    const Eigen::Matrix3d turned = placed.topLeftCorner<3, 3>();
    const Eigen::Matrix3d gram(turned.transpose() * turned);
    const Eigen::Vector4d bottom = placed.row(3).transpose() - Eigen::Vector4d::UnitW();

    return std::max({(gram - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(), std::fabs(turned.determinant() - 1.0), bottom.cwiseAbs().maxCoeff()});
}

expected<transform, std::size_t> fold_chain_end(const transform &home, std::span<const screw_axis> space_screws, const joint_vector &theta, const rigid_motion::screw_ops &screw)
{
    transform carried = transform::Identity();
    for(std::size_t joint = 0; joint < space_screws.size(); ++joint)
    {
        const auto at          = static_cast<Eigen::Index>(joint);
        const double turn      = at < theta.size() ? theta[at] : 0.0;
        const transform turned = screw.matrix_exponential_screw(space_screws[joint], turn);
        if(!is_rigid_motion(turned))
            return unexpected(joint);

        carried = transform(carried * turned);
    }

    return transform(carried * home);
}

}
