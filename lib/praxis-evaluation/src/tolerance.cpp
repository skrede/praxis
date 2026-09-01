#include "praxis/evaluation/tolerance.h"

#include <cmath>
#include <cstddef>

namespace praxis {

namespace {

std::span<const double> as_span(const Eigen::VectorXd &vector)
{
    return std::span<const double>(vector.data(), static_cast<std::size_t>(vector.size()));
}

}

bool is_approx_equal(double first, double second, double tolerance)
{
    return std::fabs(first - second) < tolerance;
}

bool is_approx_equal(std::span<const double> first, std::span<const double> second, double tolerance)
{
    if(first.size() != second.size())
        return false;
    for(std::size_t i = 0; i < first.size(); ++i)
        if(!is_approx_equal(first[i], second[i], tolerance))
            return false;
    return true;
}

bool is_approx_equal(const Eigen::VectorXd &first, const Eigen::VectorXd &second, double tolerance)
{
    if(first.size() != second.size())
        return false;
    return is_approx_equal(as_span(first), as_span(second), tolerance);
}

bool is_approx_equal(const Eigen::Matrix4d &first, const Eigen::Matrix4d &second, double tolerance)
{
    return ((first - second).cwiseAbs().array() < tolerance).all();
}

bool is_approx_equal(const Eigen::Matrix3d &first, const Eigen::Matrix3d &second, double tolerance)
{
    return ((first - second).cwiseAbs().array() < tolerance).all();
}

}
