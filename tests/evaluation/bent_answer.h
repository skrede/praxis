#ifndef HPP_GUARD_PRAXIS_TESTS_EVALUATION_BENT_ANSWER_H
#define HPP_GUARD_PRAXIS_TESTS_EVALUATION_BENT_ANSWER_H

#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/capabilities.h"

#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/generation.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <utility>
#include <type_traits>

namespace praxis::fixture {

// A decade above the bound the kind is judged against, which is the smallest displacement the sweep
// behind those bounds still reports as a difference.
inline constexpr double bend_element = 10.0 * evaluation::element_wise_tolerance;
inline constexpr double bend_radians = 10.0 * evaluation::geodesic_tolerance_radians;
inline constexpr double bend_metres  = 10.0 * evaluation::pose_tolerance_metres;
inline constexpr double bend_axis    = 10.0 * evaluation::axis_up_to_sign_tolerance;

using bend_applier = void (*)(rigid_motion::capabilities &spatial);
using bend_probe   = bool (*)(const rigid_motion::capabilities &held, const rigid_motion::capabilities &bent, evaluation::case_source &drawn);

template<typename T>
T moved(const T &value)
{
    T shifted = value;
    shifted(0, 0) += bend_element;

    return shifted;
}

// A screw axis carries an angular half in its first three components and a linear half in its last
// three, and the two are read in different units.
inline screw_axis moved_in_the_angular_half(const screw_axis &axis)
{
    screw_axis shifted = axis;
    shifted[0] += bend_axis;

    return shifted;
}

inline screw_axis moved_in_the_linear_half(const screw_axis &axis)
{
    screw_axis shifted = axis;
    shifted[3] += bend_metres;

    return shifted;
}

// Scaling a rotation would leave the group, so the answer is turned instead: it moves by
// `bend_radians` and is still orthonormal afterwards.
inline rotation turned(const rotation &r)
{
    return rotation(Eigen::AngleAxisd(bend_radians, Eigen::Vector3d::UnitX()).toRotationMatrix() * r);
}

inline transform turned(const transform &tf)
{
    transform moved_pose             = tf;
    moved_pose.topLeftCorner<3, 3>() = turned(rotation(tf.topLeftCorner<3, 3>()));

    return moved_pose;
}

inline transform displaced(const transform &tf)
{
    transform moved_pose = tf;
    moved_pose(0, 3) += bend_metres;

    return moved_pose;
}

// A refusal is passed through untouched, so a bend displaces where the reference answers and
// declines where it declines.
template<typename T, typename Displace>
expected<T, refusal> moved_or_refused(const expected<T, refusal> &read, Displace displaced_by)
{
    if(!read)
        return read;

    return displaced_by(read.value());
}

// Exact inequality rather than a tolerance: what these answer is whether a bend reached the answer
// at all, which is a question about the bend and not about any bound.
inline bool differs(const Eigen::Ref<const Eigen::MatrixXd> &held, const Eigen::Ref<const Eigen::MatrixXd> &against)
{
    return !held.cwiseEqual(against).all();
}

inline bool differs(const std::pair<Eigen::Vector3d, double> &held, const std::pair<Eigen::Vector3d, double> &against)
{
    return differs(held.first, against.first) || held.second != against.second;
}

inline bool differs(const std::pair<screw_axis, double> &held, const std::pair<screw_axis, double> &against)
{
    return differs(held.first, against.first) || held.second != against.second;
}

template<typename T>
bool differs(const expected<T, refusal> &held, const expected<T, refusal> &against)
{
    if(held.has_value() != against.has_value())
        return true;

    return held.has_value() && differs(held.value(), against.value());
}

// The arguments are not deduced, so one call draws an input once and hands the same value to both
// bindings, and the deduction comes from the two signatures alone.
template<typename R, typename... A>
bool answers_differ(R (*held)(A...), R (*against)(A...), std::type_identity_t<A>... args)
{
    return differs(held(args...), against(args...));
}

}

#endif
