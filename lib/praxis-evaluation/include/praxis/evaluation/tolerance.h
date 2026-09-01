#ifndef HPP_GUARD_PRAXIS_EVALUATION_TOLERANCE_H
#define HPP_GUARD_PRAXIS_EVALUATION_TOLERANCE_H

#include <Eigen/Core>

#include <span>

namespace praxis {

// The absolute tolerance every comparison in this repository is written against. It is
// dimensionless: it bounds the absolute difference between two numbers, and between corresponding
// elements of two sequences or two matrices.
inline constexpr double default_tolerance = 1.0e-12;

bool is_approx_equal(double first, double second, double tolerance = default_tolerance);
bool is_approx_equal(std::span<const double> first, std::span<const double> second, double tolerance = default_tolerance);
bool is_approx_equal(const Eigen::VectorXd &first, const Eigen::VectorXd &second, double tolerance = default_tolerance);
bool is_approx_equal(const Eigen::Matrix4d &first, const Eigen::Matrix4d &second, double tolerance = default_tolerance);
bool is_approx_equal(const Eigen::Matrix3d &first, const Eigen::Matrix3d &second, double tolerance = default_tolerance);

}

#endif
