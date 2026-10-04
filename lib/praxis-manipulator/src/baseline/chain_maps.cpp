#include "chain_maps.h"
#include "praxis/manipulator/baseline/kinematics.h"

#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/evaluation/tolerance.h"

#include <span>
#include <vector>
#include <cstddef>
#include <algorithm>

namespace praxis::manipulator {

namespace {

// Lynch & Park, Modern Robotics, Def. 3.24.
bool is_unit_screw(const screw_axis &s)
{
    if(!s.allFinite())
        return false;

    const double angular = s.head<3>().norm();
    if(is_approx_equal(angular, 0.0))
        return is_approx_equal(s.tail<3>().norm(), 1.0);

    return is_approx_equal(angular, 1.0);
}

bool is_admissible(std::span<const screw_axis> screws, const joint_vector &theta)
{
    return theta.allFinite() && std::ranges::all_of(screws, &is_unit_screw);
}

// Lynch & Park, Modern Robotics, sec. 5.1.1, Def. 5.1, eq. (5.11).
expected<jacobian, refusal> space_columns(std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    if(!is_admissible(space_screws, theta))
        return unexpected(refusal::degenerate);

    jacobian columns(6, theta.size());
    transform reached = transform::Identity();
    for(Eigen::Index i = 0; i < theta.size(); ++i)
    {
        const screw_axis &s                 = space_screws[static_cast<std::size_t>(i)];
        const expected<adjoint, refusal> ad = rigid_motion::adjoint_matrix_from_transform(reached);
        if(!ad)
            return unexpected(ad.error());

        columns.col(i) = *ad * s;
        reached        = reached * rigid_motion::matrix_exponential_screw(s, theta[i]);
    }

    return columns;
}

}

bool is_a_rigid_motion(const transform &tf)
{
    const rotation r = tf.block<3, 3>(0, 0);

    return is_approx_equal(rotation(r.transpose() * r), rotation::Identity()) && is_approx_equal(r.determinant(), 1.0) &&
            is_approx_equal((tf.row(3) - Eigen::RowVector4d::UnitW()).cwiseAbs().maxCoeff(), 0.0);
}

bool is_a_rigid_motion(const rigid_motion::frame_ops &frames, const transform &tf)
{
    const rotation seen = frames.rotation_matrix_from_transform(tf);

    return is_a_rigid_motion(tf) && is_approx_equal(rotation(seen.transpose() * seen), rotation::Identity()) && is_approx_equal(seen.determinant(), 1.0);
}

bool is_admitted(const screw_chain &chain)
{
    return !chain.space_screws.empty() && chain.home.allFinite() && is_a_rigid_motion(chain.home) && std::ranges::all_of(chain.space_screws, &is_unit_screw);
}

expected<std::vector<screw_axis>, refusal> to_body_screws(const rigid_motion::screw_ops &screw, const transform &m, std::span<const screw_axis> space_screws)
{
    const rotation transposed                     = m.block<3, 3>(0, 0).transpose();
    const expected<adjoint, refusal> inverse_home = screw.adjoint_matrix_from_rotation_position(transposed, -(transposed * m.block<3, 1>(0, 3)));
    if(!inverse_home)
        return unexpected(inverse_home.error());

    std::vector<screw_axis> body;
    body.reserve(space_screws.size());
    for(const screw_axis &s : space_screws)
        body.push_back(*inverse_home * s);

    return body;
}

transform exponential_product(std::span<const screw_axis> screws, const joint_vector &theta)
{
    transform product = transform::Identity();
    for(Eigen::Index i = 0; i < theta.size(); ++i)
        product = product * rigid_motion::matrix_exponential_screw(screws[static_cast<std::size_t>(i)], theta[i]);

    return product;
}

// Lynch & Park, Modern Robotics, sec. 5.1.2, Def. 5.4, eq. (5.18).
expected<jacobian, refusal> body_columns(std::span<const screw_axis> body, const joint_vector &theta)
{
    if(theta.size() != static_cast<Eigen::Index>(body.size()))
        return unexpected(refusal::unsupported_input);
    if(!is_admissible(body, theta))
        return unexpected(refusal::degenerate);

    jacobian columns(6, theta.size());
    transform reached = transform::Identity();
    for(Eigen::Index i = theta.size() - 1; i >= 0; --i)
    {
        const screw_axis &b                 = body[static_cast<std::size_t>(i)];
        const expected<adjoint, refusal> ad = rigid_motion::adjoint_matrix_from_transform(reached);
        if(!ad)
            return unexpected(ad.error());

        columns.col(i) = *ad * b;
        reached        = reached * rigid_motion::matrix_exponential_screw(b, -theta[i]);
    }

    return columns;
}

// Lynch & Park, Modern Robotics, sec. 4.1.1, eq. (4.14).
expected<transform, refusal> forward_kinematics(const rigid_motion::screw_ops &, const transform &m, std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    if(theta.size() != static_cast<Eigen::Index>(space_screws.size()))
        return unexpected(refusal::unsupported_input);
    if(!is_a_rigid_motion(m))
        return unexpected(refusal::degenerate);
    if(space_screws.empty())
        return transform(m);
    if(!m.allFinite() || !is_admissible(space_screws, theta))
        return unexpected(refusal::degenerate);

    return transform(exponential_product(space_screws, theta) * m);
}

// Lynch & Park, Modern Robotics, sec. 4.1.3, eq. (4.16).
expected<transform, refusal> body_forward_kinematics(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const transform &m,
                                                     std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    if(theta.size() != static_cast<Eigen::Index>(space_screws.size()))
        return unexpected(refusal::unsupported_input);
    if(!is_a_rigid_motion(frames, m))
        return unexpected(refusal::degenerate);
    if(space_screws.empty())
        return transform(m);

    const expected<std::vector<screw_axis>, refusal> body_screws = to_body_screws(screw, m, space_screws);
    if(!body_screws)
        return unexpected(body_screws.error());
    if(!m.allFinite() || !is_admissible(*body_screws, theta))
        return unexpected(refusal::degenerate);

    return transform(m * exponential_product(*body_screws, theta));
}

// The body screws are the space screws seen from the home pose, so a home pose that is not a rigid
// motion has no inverse adjoint to see them through.
expected<std::vector<screw_axis>, refusal> body_screws_from_space(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const transform &m,
                                                                  std::span<const screw_axis> space_screws)
{
    if(!is_a_rigid_motion(frames, m))
        return unexpected(refusal::degenerate);

    return to_body_screws(screw, m, space_screws);
}

expected<jacobian, refusal> space_jacobian(const rigid_motion::screw_ops &, std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    if(theta.size() != static_cast<Eigen::Index>(space_screws.size()))
        return unexpected(refusal::unsupported_input);
    if(space_screws.empty())
        return jacobian(jacobian::Zero(6, 0));

    return space_columns(space_screws, theta);
}

expected<jacobian, refusal> body_jacobian(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const forward_kinematics_ops &forward, const transform &m,
                                          std::span<const screw_axis> space_screws, const joint_vector &theta)
{
    if(theta.size() != static_cast<Eigen::Index>(space_screws.size()))
        return unexpected(refusal::unsupported_input);
    if(!is_a_rigid_motion(frames, m))
        return unexpected(refusal::degenerate);
    if(space_screws.empty())
        return jacobian(jacobian::Zero(6, 0));

    const expected<std::vector<screw_axis>, refusal> body_screws = forward.body_screws_from_space(screw, frames, m, space_screws);
    if(!body_screws)
        return unexpected(body_screws.error());

    return body_columns(*body_screws, theta);
}

}
