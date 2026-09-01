#include "praxis/evaluation/generation.h"

#include <Eigen/Geometry>

namespace praxis::evaluation {

Eigen::Matrix3d case_source::rotation_member()
{
    const double radians        = angle_radians();
    const Eigen::Vector3d about = unit_direction();

    return Eigen::AngleAxisd(radians, about).toRotationMatrix();
}

Eigen::Matrix3d case_source::orthonormal_triple()
{
    return rotation_member();
}

Eigen::Matrix4d case_source::transform_member()
{
    const Eigen::Matrix3d turned = rotation_member();
    const Eigen::Vector3d at     = position_metres();

    Eigen::Matrix4d pose   = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = turned;
    pose.block<3, 1>(0, 3) = at;

    return pose;
}

Eigen::Matrix3d case_source::skew_symmetric_member()
{
    const Eigen::Vector3d v = normal_triple();

    Eigen::Matrix3d crossed;
    crossed << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;

    return crossed;
}

Eigen::Vector<double, 6> case_source::unit_twist()
{
    Eigen::Vector<double, 6> axis = Eigen::Vector<double, 6>::Zero();

    if(m_spread == spread::near_singular || half_the_time())
    {
        axis.tail<3>() = unit_direction();

        return axis;
    }

    axis.head<3>() = unit_direction();
    axis.tail<3>() = normal_triple();

    return axis;
}

Eigen::Vector<double, 6> case_source::twist_member()
{
    Eigen::Vector<double, 6> drawn;

    drawn.head<3>() = normal_triple();
    drawn.tail<3>() = normal_triple();

    return drawn;
}

}
