#include "praxis/evaluation/tolerance.h"
#include "praxis/evaluation/comparators.h"

#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <cstdint>

namespace praxis::evaluation {

namespace {

// The deviation from unit length an angular part may carry and still name a direction.
constexpr double axis_unitness_bound = 1.0e-1;

enum class axis_reading : std::uint8_t
{
    collapsed,
    unit,
    neither,
};

axis_reading reading_of(const Eigen::Vector3d &axis)
{
    const double length = axis.norm();

    if(length < default_tolerance)
        return axis_reading::collapsed;

    return std::fabs(length - 1.0) <= axis_unitness_bound ? axis_reading::unit : axis_reading::neither;
}

bool either_names_no_direction(const Eigen::Vector3d &first, const Eigen::Vector3d &second)
{
    return reading_of(first) == axis_reading::neither || reading_of(second) == axis_reading::neither;
}

// A collapsed angular part is a translation rather than a direction, and is handed on as it stands.
Eigen::Vector3d as_a_direction(const Eigen::Vector3d &axis)
{
    return reading_of(axis) == axis_reading::unit ? Eigen::Vector3d(axis.normalized()) : axis;
}

// The linear half carries the pitch and has no unit length to be held to.
Eigen::Vector<double, 6> as_a_unit_axis(const Eigen::Vector<double, 6> &axis)
{
    Eigen::Vector<double, 6> handed(axis);

    handed.head<3>() = as_a_direction(axis.head<3>());

    return handed;
}

Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d m;

    m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;

    return m;
}

// An axis whose norm has collapsed cannot be normalized, and an angle that has turns nothing, so
// both answer the element that leaves everything where it is.
Eigen::Matrix3d turned_by(const Eigen::Vector3d &axis, double theta_radians)
{
    if(axis.norm() < default_tolerance || std::fabs(theta_radians) < default_tolerance)
        return Eigen::Matrix3d::Identity();

    return Eigen::AngleAxisd(theta_radians, axis.normalized()).toRotationMatrix();
}

// The closed form over an axis split into its angular half w and its linear half v, with w taken as
// a unit vector. Lynch & Park, Modern Robotics, section 3.3.3. Where w has collapsed the series is
// singular and the element is a translation of theta along v.
Eigen::Matrix4d moved_by(const Eigen::Vector<double, 6> &axis, double theta_radians)
{
    const Eigen::Vector3d w(axis.head<3>());
    const Eigen::Vector3d v(axis.tail<3>());
    const Eigen::Matrix3d unit(Eigen::Matrix3d::Identity());
    Eigen::Matrix4d element(Eigen::Matrix4d::Identity());

    if(w.norm() < default_tolerance)
    {
        element.block<3, 1>(0, 3) = v * theta_radians;

        return element;
    }

    const Eigen::Matrix3d skew(skew_symmetric(w));
    const double cosine = std::cos(theta_radians);
    const double sine   = std::sin(theta_radians);

    element.block<3, 3>(0, 0) = unit + sine * skew + (1.0 - cosine) * skew * skew;
    element.block<3, 1>(0, 3) = (unit * theta_radians + (1.0 - cosine) * skew + (theta_radians - sine) * skew * skew) * v;

    return element;
}

}

residual log_up_to_branch_rotation_residual(const Eigen::Vector3d &first_axis, double first_theta_radians, const Eigen::Vector3d &second_axis, double second_theta_radians)
{
    if(either_names_no_direction(first_axis, second_axis))
        return residual{residual_kind::log_up_to_branch, std::numeric_limits<double>::infinity(), 0.0};

    const residual between = geodesic_residual(turned_by(as_a_direction(first_axis), first_theta_radians), turned_by(as_a_direction(second_axis), second_theta_radians));

    return residual{residual_kind::log_up_to_branch, between.magnitude, between.linear_error_metres};
}

// The closed form's translation block is a series in the angular part, so a non-unit angular part
// leaves neither half of the element built from a legitimate input.
residual log_up_to_branch_pose_residual(const Eigen::Vector<double, 6> &first_axis, double first_theta_radians, const Eigen::Vector<double, 6> &second_axis, double second_theta_radians)
{
    if(either_names_no_direction(first_axis.head<3>(), second_axis.head<3>()))
        return residual{residual_kind::log_up_to_branch, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};

    const residual between = pose_residual(moved_by(as_a_unit_axis(first_axis), first_theta_radians), moved_by(as_a_unit_axis(second_axis), second_theta_radians));

    return residual{residual_kind::log_up_to_branch, between.magnitude, between.linear_error_metres};
}

}
