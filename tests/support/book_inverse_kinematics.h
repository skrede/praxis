#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_INVERSE_KINEMATICS_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_BOOK_INVERSE_KINEMATICS_H

#include "book_chain_maps.h"

#include "praxis/manipulator/kinematics.h"

#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>

#include <array>
#include <string>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <string_view>

// Lynch & Park, Modern Robotics, sec. 6.2.2: the Newton-Raphson iteration on the body twist, the
// step of eq. (6.6) through the pseudoinverse, and its space form through V_s = [Ad_Tsb] V_b.
namespace praxis::tests::book {

enum class step : std::uint8_t
{
    cod,
    svd,
    normal
};

enum class twist_frame : std::uint8_t
{
    body,
    space
};

enum class helpers : std::uint8_t
{
    handed,
    text
};

enum class iteration_cap : std::uint8_t
{
    handed,
    twenty
};

struct handed_maps
{
    const rigid_motion::screw_ops &screw;
    const manipulator::forward_kinematics_ops &forward;
    const manipulator::differential_kinematics_ops &differential;
    const manipulator::screw_chain &chain;
};

template<helpers H>
expected<transform, refusal> reached(const handed_maps &maps, const joint_vector &theta)
{
    return H == helpers::handed ? maps.forward.forward_kinematics(maps.screw, maps.chain.home, maps.chain.space_screws, theta)
                                : forward_kinematics(maps.screw, maps.chain.home, maps.chain.space_screws, theta);
}

// [V_b] = log(T_sb^-1 T_sd), angular half first.
template<twist_frame F, helpers H>
expected<twist, refusal> error_twist(const handed_maps &maps, const transform &desired, const joint_vector &theta)
{
    const expected<transform, refusal> tsb = reached<H>(maps, theta);
    if(!tsb)
        return unexpected(tsb.error());
    const transform tbd                             = inverse_of(*tsb) * desired;
    const expected<pose_reading, refusal> logarithm = H == helpers::handed ? maps.screw.matrix_logarithm_se3(tbd) : matrix_logarithm_se3(tbd);
    if(!logarithm)
        return unexpected(logarithm.error());
    const twist vb = logarithm->first * logarithm->second;
    if constexpr(F == twist_frame::body)
        return vb;
    const expected<adjoint, refusal> ad = H == helpers::handed ? maps.screw.adjoint_matrix_from_transform(*tsb) : expected<adjoint, refusal>(adjoint_of(*tsb));
    if(!ad)
        return unexpected(ad.error());
    return twist(*ad * vb);
}

// eq. (5.22): J_b = [Ad_Tbs] J_s
inline expected<jacobian, refusal> handed_body_jacobian(const handed_maps &maps, const joint_vector &theta)
{
    const expected<transform, refusal> tsb = reached<helpers::handed>(maps, theta);
    const expected<jacobian, refusal> js   = maps.differential.space_jacobian(maps.screw, maps.chain.space_screws, theta);
    if(!tsb || !js)
        return unexpected(refusal::no_solution);
    const expected<adjoint, refusal> ad = maps.screw.adjoint_matrix_from_transform(inverse_of(*tsb));
    if(!ad)
        return unexpected(ad.error());
    return jacobian(*ad * *js);
}

template<twist_frame F, helpers H>
expected<jacobian, refusal> jacobian_at(const handed_maps &maps, const joint_vector &theta)
{
    if constexpr(F == twist_frame::space && H == helpers::handed)
        return maps.differential.space_jacobian(maps.screw, maps.chain.space_screws, theta);
    else if constexpr(F == twist_frame::space)
        return space_jacobian(maps.screw, maps.chain.space_screws, theta);
    else if constexpr(H == helpers::handed)
        return handed_body_jacobian(maps, theta);
    else
        return body_jacobian(maps.screw, rigid_motion::frame_ops{}, manipulator::forward_kinematics_ops{}, maps.chain.home, maps.chain.space_screws, theta);
}

// The full-rank pseudoinverse of sec. 6.2.2: (J^T J)^-1 J^T for a tall J, J^T (J J^T)^-1 otherwise.
template<step S>
joint_vector step_along(const jacobian &j, const twist &v)
{
    if constexpr(S == step::cod)
        return Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>(j).pseudoInverse() * v;
    else if constexpr(S == step::svd)
        return Eigen::JacobiSVD<Eigen::MatrixXd>(j, Eigen::ComputeThinU | Eigen::ComputeThinV).solve(v);
    else if(j.cols() < j.rows())
        return (j.transpose() * j).inverse() * j.transpose() * v;
    else
        return j.transpose() * (j * j.transpose()).inverse() * v;
}

inline bool outside(const twist &v, const manipulator::solver_parameters &parameters)
{
    return v.head<3>().norm() > parameters.orientation_tol || v.tail<3>().norm() > parameters.position_tol;
}

template<step S, twist_frame F, helpers H>
expected<joint_vector, refusal> iterated(const handed_maps &maps, const transform &desired, const joint_vector &j0, const manipulator::solver_parameters &parameters, std::uint32_t cap)
{
    joint_vector theta         = j0;
    expected<twist, refusal> v = error_twist<F, H>(maps, desired, theta);
    for(std::uint32_t i = 0; v && outside(*v, parameters) && i < cap; ++i)
    {
        const expected<jacobian, refusal> j = jacobian_at<F, H>(maps, theta);
        if(!j)
            return unexpected(refusal::no_solution);
        theta += step_along<S>(*j, *v);
        v = error_twist<F, H>(maps, desired, theta);
    }
    if(!v || outside(*v, parameters))
        return unexpected(refusal::no_solution);
    return theta;
}

template<step S, twist_frame F, helpers H, iteration_cap C>
expected<void, refusal> ikin(const rigid_motion::screw_ops &screw, const manipulator::forward_kinematics_ops &forward, const manipulator::differential_kinematics_ops &differential,
                             const manipulator::screw_chain &chain, const transform &desired, const joint_vector &j0, const manipulator::solver_parameters &parameters,
                             manipulator::ik_result &answer)
{
    if(j0.size() != static_cast<Eigen::Index>(chain.joint_count()))
        return unexpected(refusal::unsupported_input);
    if(!is_rigid(desired))
        return unexpected(refusal::degenerate);
    if(!j0.allFinite())
        return unexpected(refusal::no_solution);
    const std::uint32_t cap                     = C == iteration_cap::handed ? parameters.max_iterations_per_attempt : 20u;
    const expected<joint_vector, refusal> theta = iterated<S, F, H>(handed_maps{screw, forward, differential, chain}, desired, j0, parameters, cap);
    if(!theta)
        return unexpected(theta.error());
    answer.solutions.push_back(*theta);
    return {};
}

using inverse_kinematics_slot = decltype(manipulator::inverse_kinematics_ops::inverse_kinematics);

struct literal_form
{
    twist_frame frame;
    step stepping;
    helpers source;
    iteration_cap cap;
    inverse_kinematics_slot solve;
};

template<std::size_t I>
constexpr literal_form literal_form_at()
{
    constexpr twist_frame frame = static_cast<twist_frame>(I / 12u);
    constexpr step stepping     = static_cast<step>(I / 4u % 3u);
    constexpr helpers source    = static_cast<helpers>(I / 2u % 2u);
    constexpr iteration_cap cap = static_cast<iteration_cap>(I % 2u);
    return literal_form{frame, stepping, source, cap, &ikin<stepping, frame, source, cap>};
}

template<std::size_t... I>
constexpr std::array<literal_form, sizeof...(I)> literal_forms_of(std::index_sequence<I...>)
{
    return {literal_form_at<I>()...};
}

inline constexpr std::array<literal_form, 24> literal_forms = literal_forms_of(std::make_index_sequence<24>{});

inline std::string form_label(const literal_form &form)
{
    constexpr std::array<std::string_view, 2> frames{"body", "space"};
    constexpr std::array<std::string_view, 3> steps{"cod", "svd", "normal"};
    constexpr std::array<std::string_view, 2> sources{"handed", "text"};
    constexpr std::array<std::string_view, 2> caps{"handed", "twenty"};
    return std::string(frames.at(static_cast<std::size_t>(form.frame))) + "." + std::string(steps.at(static_cast<std::size_t>(form.stepping))) + "." +
            std::string(sources.at(static_cast<std::size_t>(form.source))) + "." + std::string(caps.at(static_cast<std::size_t>(form.cap)));
}

}

#endif
