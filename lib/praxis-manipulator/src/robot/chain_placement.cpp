#include "robot/chain_placement.h"

#include "praxis/rigid_motion/types.h"

#include <Eigen/Core>

#include <span>
#include <vector>
#include <cstddef>

namespace praxis::manipulator {

namespace {

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

}
