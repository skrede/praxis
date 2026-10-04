#include "book_logarithm.h"
#include "group_membership.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include <cmath>
#include <numbers>

namespace praxis::rigid_motion {

namespace {

using rotation_reading  = std::pair<Eigen::Vector3d, double>;
using logarithm_reading = rotation_reading (*)(const rotation &r);

// Lynch & Park, Modern Robotics, sec. 3.2.3.3, eqs. (3.58)-(3.60): each is column i of R + I over
// sqrt(2(1 + r_ii)); i is the largest diagonal entry.
Eigen::Vector3d half_turn_axis(const rotation &r)
{
    Eigen::Index i = 0;
    r.diagonal().maxCoeff(&i);

    return (r + rotation::Identity()).col(i) / std::sqrt(2.0 * (1.0 + r(i, i)));
}

// Lynch & Park, Modern Robotics, sec. 3.2.3.3, eqs. (3.58)-(3.61): the cases are read from the
// computed (tr R - 1)/2, case (a) leaves the axis undefined, answered here as x, and the eq. (3.61)
// axis is held to the unit length the section states for it.
rotation_reading book_rotation_logarithm(const rotation &r)
{
    const double cosine = (r.trace() - 1.0) / 2.0;
    if(cosine >= 1.0)
        return {Eigen::Vector3d::UnitX(), 0.0};
    if(cosine <= -1.0)
        return {half_turn_axis(r), std::numbers::pi};

    const double theta         = std::acos(cosine);
    const Eigen::Vector3d axis = from_skew_symmetric((r - r.transpose()) / (2.0 * std::sin(theta)));

    return {axis.normalized(), theta};
}

// Cases (a) and (b) of sec. 3.2.3.3, with case (c) beyond eq. (3.61): theta = atan2(||vee(R - R^T)||/2,
// (tr R - 1)/2), the axis that vector's direction. Only a symmetric R is case (a) or (b) when c > -1.
rotation_reading rotation_logarithm(const rotation &r)
{
    const double cosine = (r.trace() - 1.0) / 2.0;
    if(cosine <= -1.0)
        return {half_turn_axis(r), std::numbers::pi};

    const Eigen::Vector3d twice_sine_axis = from_skew_symmetric(r - r.transpose());
    const double twice_sine               = twice_sine_axis.norm();
    if(twice_sine == 0.0)
        return cosine > 0.0 ? rotation_reading{Eigen::Vector3d::UnitX(), 0.0} : rotation_reading{half_turn_axis(r), std::numbers::pi};

    return {twice_sine_axis / twice_sine, std::atan2(twice_sine / 2.0, cosine)};
}

// Lynch & Park, Modern Robotics, sec. 3.3.3.2: case (a) is a rotation logarithm of no turn, and p = 0
// there names no axis, answered as a translation along x; case (b) is eqs. (3.91)-(3.92).
std::pair<screw_axis, double> screw_logarithm(const rotation &r, const Eigen::Vector3d &p, logarithm_reading logarithm)
{
    screw_axis s;
    const auto [w, theta] = logarithm(r);
    if(theta == 0.0)
    {
        const double distance = p.norm();
        s << Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX();
        if(distance != 0.0)
            s.tail<3>() = p / distance;
        return {s, distance};
    }

    const matrix3 m         = skew_symmetric(w);
    const matrix3 g_inverse = matrix3::Identity() / theta - m / 2.0 + (1.0 / theta - 0.5 / std::tan(theta / 2.0)) * m * m;
    s << w, g_inverse * p;

    return {s, theta};
}

}

expected<std::pair<Eigen::Vector3d, double>, refusal> matrix_logarithm_so3(const rotation &r)
{
    if(!is_a_rotation(r))
        return unexpected(refusal::degenerate);

    return rotation_logarithm(r);
}

expected<std::pair<screw_axis, double>, refusal> matrix_logarithm_se3_rp(const rotation &r, const Eigen::Vector3d &p)
{
    if(!is_a_rigid_motion(r, p))
        return unexpected(refusal::degenerate);

    return screw_logarithm(r, p, &rotation_logarithm);
}

expected<std::pair<screw_axis, double>, refusal> matrix_logarithm_se3(const transform &tf)
{
    if(!is_a_rigid_motion(tf))
        return unexpected(refusal::degenerate);

    return screw_logarithm(tf.block<3, 3>(0, 0), tf.block<3, 1>(0, 3), &rotation_logarithm);
}

expected<std::pair<Eigen::Vector3d, double>, refusal> book::matrix_logarithm_so3(const rotation &r)
{
    if(!is_a_rotation(r))
        return unexpected(refusal::degenerate);

    return book_rotation_logarithm(r);
}

expected<std::pair<screw_axis, double>, refusal> book::matrix_logarithm_se3_rp(const rotation &r, const Eigen::Vector3d &p)
{
    if(!is_a_rigid_motion(r, p))
        return unexpected(refusal::degenerate);

    return screw_logarithm(r, p, &book_rotation_logarithm);
}

expected<std::pair<screw_axis, double>, refusal> book::matrix_logarithm_se3(const transform &tf)
{
    if(!is_a_rigid_motion(tf))
        return unexpected(refusal::degenerate);

    return screw_logarithm(tf.block<3, 3>(0, 0), tf.block<3, 1>(0, 3), &book_rotation_logarithm);
}

}
