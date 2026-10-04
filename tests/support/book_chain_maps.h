#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_CHAIN_MAPS_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_CHAIN_MAPS_H

#include "book_screw_maps.h"

#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/trajectory/capabilities.h"

#include <span>
#include <vector>
#include <cstddef>
#include <algorithm>

// Lynch & Park, Modern Robotics, chapters 4, 5 and 9, transcribed from the text alone over the chapter 3
// transcription beside it: every product of exponentials as printed, and every Jacobian column
// recomputed from its own product. The refusal ahead of a formula is the slot's contract.
namespace praxis::tests::book {

using manipulator::jacobian;
using manipulator::joint_vector;

inline bool is_a_unit_screw(const screw_axis &s)
{
    if(!s.allFinite())
        return false;
    if(is_approx_equal(s.head<3>().norm(), 0.0))
        return is_approx_equal(s.tail<3>().norm(), 1.0);
    return is_approx_equal(s.head<3>().norm(), 1.0);
}

inline bool is_admissible(std::span<const screw_axis> screws, const joint_vector &theta)
{
    return theta.allFinite() && std::ranges::all_of(screws, &is_a_unit_screw);
}

inline bool is_as_wide(std::span<const screw_axis> screws, const joint_vector &theta)
{
    return theta.size() == static_cast<Eigen::Index>(screws.size());
}

// eq. (3.64)
inline transform inverse_of(const transform &tf)
{
    transform inverse         = transform::Identity();
    inverse.block<3, 3>(0, 0) = tf.block<3, 3>(0, 0).transpose();
    inverse.block<3, 1>(0, 3) = -tf.block<3, 3>(0, 0).transpose() * tf.block<3, 1>(0, 3);
    return inverse;
}

inline adjoint adjoint_of(const transform &tf)
{
    return adjoint_of(tf.block<3, 3>(0, 0), tf.block<3, 1>(0, 3));
}

// sec. 4.1.3: B_i = [Ad_{M^-1}] S_i
inline std::vector<screw_axis> body_screws(const transform &m, std::span<const screw_axis> space_screws)
{
    std::vector<screw_axis> body;
    for(const screw_axis &s : space_screws)
        body.push_back(adjoint_of(inverse_of(m)) * s);
    return body;
}

// eq. (4.14): T = e^[S1]theta1 ... e^[Sn]thetan M
inline expected<transform, refusal> forward_kinematics(const rigid_motion::screw_ops &, const transform &m, std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    if(!is_as_wide(space_screws, theta))
        return unexpected(refusal::unsupported_input);
    if(!is_rigid(m) || (!space_screws.empty() && !is_admissible(space_screws, theta)))
        return unexpected(refusal::degenerate);
    transform t = transform::Identity();
    for(Eigen::Index i = 0; i < theta.size(); ++i)
        t = t * matrix_exponential_screw(space_screws[static_cast<std::size_t>(i)], theta[i]);
    return transform(t * m);
}

// eq. (4.16): T = M e^[B1]theta1 ... e^[Bn]thetan
inline expected<transform, refusal> body_forward_kinematics(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &, const transform &m,
                                                            std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    if(!is_as_wide(space_screws, theta))
        return unexpected(refusal::unsupported_input);
    if(!is_rigid(m))
        return unexpected(refusal::degenerate);
    const std::vector<screw_axis> b = body_screws(m, space_screws);
    if(!b.empty() && !is_admissible(b, theta))
        return unexpected(refusal::degenerate);
    transform t = m;
    for(Eigen::Index i = 0; i < theta.size(); ++i)
        t = t * matrix_exponential_screw(b[static_cast<std::size_t>(i)], theta[i]);
    return t;
}

inline expected<std::vector<screw_axis>, refusal> body_screws_from_space(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &, const transform &m,
                                                                         std::span<const screw_axis> space_screws)
{
    if(!is_rigid(m))
        return unexpected(refusal::degenerate);
    return body_screws(m, space_screws);
}

// Def. 5.1, eq. (5.11): J_si = [Ad_{e^[S1]theta1 ... e^[S(i-1)]theta(i-1)}] S_i
inline expected<jacobian, refusal> space_jacobian(const rigid_motion::screw_ops &, std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    if(!is_as_wide(space_screws, theta))
        return unexpected(refusal::unsupported_input);
    if(!space_screws.empty() && !is_admissible(space_screws, theta))
        return unexpected(refusal::degenerate);
    jacobian j(6, theta.size());
    for(Eigen::Index i = 0; i < theta.size(); ++i)
    {
        transform t = transform::Identity();
        for(Eigen::Index k = 0; k < i; ++k)
            t = t * matrix_exponential_screw(space_screws[static_cast<std::size_t>(k)], theta[k]);
        j.col(i) = adjoint_of(t) * space_screws[static_cast<std::size_t>(i)];
    }
    return j;
}

// Def. 5.4, eq. (5.18): J_bi = [Ad_{e^-[Bn]thetan ... e^-[B(i+1)]theta(i+1)}] B_i
inline expected<jacobian, refusal> body_jacobian(const rigid_motion::screw_ops &, const rigid_motion::frame_ops &, const manipulator::forward_kinematics_ops &, const transform &m,
                                                 std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    if(!is_as_wide(space_screws, theta))
        return unexpected(refusal::unsupported_input);
    if(!is_rigid(m))
        return unexpected(refusal::degenerate);
    const std::vector<screw_axis> b = body_screws(m, space_screws);
    if(!b.empty() && !is_admissible(b, theta))
        return unexpected(refusal::degenerate);
    jacobian j(6, theta.size());
    for(Eigen::Index i = 0; i < theta.size(); ++i)
    {
        transform t = transform::Identity();
        for(Eigen::Index k = theta.size() - 1; k > i; --k)
            t = t * matrix_exponential_screw(b[static_cast<std::size_t>(k)], -theta[k]);
        j.col(i) = adjoint_of(t) * b[static_cast<std::size_t>(i)];
    }
    return j;
}

// eq. (9.6): X(s) = X_start exp(log(X_start^-1 X_end) s)
inline expected<transform, refusal> screw_path(const transform &start, const transform &end, double s)
{
    if(!is_rigid(start) || !is_rigid(end))
        return unexpected(refusal::degenerate);
    const transform relative = inverse_of(start) * end;
    const auto [axis, theta] = pose_logarithm(relative.block<3, 3>(0, 0), relative.block<3, 1>(0, 3));
    return transform(start * matrix_exponential_screw(axis, theta * s));
}

// eq. (9.8): p(s) = p_start + s(p_end - p_start), R(s) = R_start exp(log(R_start^T R_end) s)
inline expected<transform, refusal> decoupled_path(const transform &start, const transform &end, double s)
{
    if(!is_rigid(start) || !is_rigid(end))
        return unexpected(refusal::degenerate);
    const rotation r_start   = start.block<3, 3>(0, 0);
    const auto [axis, theta] = rotation_logarithm(rotation(r_start.transpose() * end.block<3, 3>(0, 0)));
    transform x              = transform::Identity();
    x.block<3, 3>(0, 0)      = r_start * matrix_exponential_so3(axis, theta * s);
    x.block<3, 1>(0, 3)      = start.block<3, 1>(0, 3) + s * (end.block<3, 1>(0, 3) - start.block<3, 1>(0, 3));
    return x;
}

inline manipulator::capabilities chain_maps()
{
    manipulator::capabilities book  = manipulator::baseline();
    book.fk.forward_kinematics      = &forward_kinematics;
    book.fk.body_forward_kinematics = &body_forward_kinematics;
    book.fk.body_screws_from_space  = &body_screws_from_space;
    book.dk.space_jacobian          = &space_jacobian;
    book.dk.body_jacobian           = &body_jacobian;
    return book;
}

inline trajectory::capabilities path_maps()
{
    trajectory::capabilities book = trajectory::baseline();
    book.path.screw               = &screw_path;
    book.path.decoupled           = &decoupled_path;
    return book;
}

}

#endif
