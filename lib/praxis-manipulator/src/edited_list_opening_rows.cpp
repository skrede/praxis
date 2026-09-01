#include "praxis/manipulator/edited_list_rows.h"

#include "praxis/rigid_motion/angles.h"

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <vector>
#include <cstddef>
#include <numbers>

namespace praxis::manipulator {

namespace {

constexpr std::size_t opening_waypoints = 4;

constexpr double sweep_degrees = 75.0;
constexpr double dip_degrees   = 60.0;

// The position is metres in the space frame and the angles are degrees, read in the axis order an
// edited pose opens at.
struct opening_pose
{
    double x;
    double y;
    double z;
    double first;
    double second;
    double third;
};

constexpr std::array shipped_poses{
        opening_pose{0.01, 0.47, 0.30, 60.0, 160.0, -90.0},
        opening_pose{0.20, 0.31, 0.11, 20.0, -140.0, -90.0},
        opening_pose{-0.10, 0.28, 0.18, 60.0, -140.0, -90.0},
        opening_pose{0.19, 0.30, 0.26, 20.0, -170.0, -90.0},
};

// Row `row` of `count`: the first joint sweeps evenly across its swing while the second dips and
// returns to where it started, so no two rows coincide and the rows do not lie on one line. Every
// joint beyond the second stands at its home value.
joint_vector opening_row(std::size_t joints, std::size_t row, std::size_t count)
{
    const double along = count < 2u ? 0.0 : static_cast<double>(row) / static_cast<double>(count - 1u);

    joint_vector taken = joint_vector::Zero(static_cast<Eigen::Index>(joints));
    if(joints > 0u)
        taken[0] = (2.0 * along - 1.0) * sweep_degrees * radians_per_degree;
    if(joints > 1u)
        taken[1] = -std::sin(along * std::numbers::pi) * dip_degrees * radians_per_degree;

    return taken;
}

edited_pose posed_at(const opening_pose &named)
{
    edited_pose taken;
    taken.position      = Eigen::Vector3d(named.x, named.y, named.z).cast<float>();
    taken.euler_degrees = Eigen::Vector3d(named.first, named.second, named.third).cast<float>();

    return taken;
}

}

std::vector<joint_vector> list_row_traits<joint_vector>::opening_rows(std::size_t joints)
{
    std::vector<joint_vector> rows;
    for(std::size_t row = 0; joints != 0u && row < opening_waypoints; ++row)
        rows.push_back(opening_row(joints, row, opening_waypoints));

    return rows;
}

// A pose is six numbers in the space frame whatever the arm is, so the joint count does not enter.
std::vector<edited_pose> list_row_traits<edited_pose>::opening_rows(std::size_t)
{
    std::vector<edited_pose> rows;
    for(const opening_pose &named : shipped_poses)
        rows.push_back(posed_at(named));

    return rows;
}

}
