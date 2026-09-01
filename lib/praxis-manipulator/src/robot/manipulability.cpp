#include "robot/manipulability.h"

#include <Eigen/SVD>
#include <Eigen/Core>

#include <optional>

namespace praxis::manipulator {

namespace {

// A quaternion is composed from this basis downstream and a reflection is not a rotation, so the
// third column is negated where the columns the decomposition answered are left-handed.
Eigen::Matrix3d right_handed(Eigen::Matrix3d axes)
{
    if(axes.determinant() < 0.0)
        axes.col(2) = -axes.col(2);

    return axes;
}

std::optional<double> largest_over_smallest(const Eigen::Vector3d &values)
{
    if(values(2) <= 0.0)
        return std::nullopt;

    return values(0) / values(2);
}

}

expected<manipulability_ellipsoid, refusal> ellipsoid_of(const Eigen::Ref<const Eigen::MatrixXd> &block)
{
    if(block.rows() != 3 || block.cols() < 3)
        return unexpected(refusal::unsupported_input);

    if(!block.allFinite())
        return unexpected(refusal::degenerate);

    const Eigen::JacobiSVD<Eigen::MatrixXd> taken(block, Eigen::ComputeFullU);

    manipulability_ellipsoid answered{};
    answered.singular_values = taken.singularValues().head<3>().cwiseMax(0.0);
    answered.principal_axes  = right_handed(taken.matrixU());

    if(!answered.singular_values.allFinite() || !answered.principal_axes.allFinite())
        return unexpected(refusal::degenerate);

    answered.measure   = answered.singular_values.prod();
    answered.condition = largest_over_smallest(answered.singular_values);

    return answered;
}

jacobian_manipulability manipulability_of(const expected<jacobian, refusal> &taken)
{
    if(!taken)
        return jacobian_manipulability{unexpected(taken.error()), unexpected(taken.error())};

    return jacobian_manipulability{ellipsoid_of(taken->topRows(3)), ellipsoid_of(taken->bottomRows(3))};
}

}
