#ifndef HPP_GUARD_PRAXIS_TESTS_EVALUATION_BOOK_LOGARITHM_H
#define HPP_GUARD_PRAXIS_TESTS_EVALUATION_BOOK_LOGARITHM_H

#include "bent_answer.h"

#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/capabilities.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <utility>

namespace praxis::fixture {

using rotation_reading = std::pair<Eigen::Vector3d, double>;
using pose_reading     = std::pair<screw_axis, double>;

using rotation_logarithm_slot  = expected<rotation_reading, refusal> (*)(const rotation &r);
using pose_logarithm_slot      = expected<pose_reading, refusal> (*)(const rotation &r, const Eigen::Vector3d &p);
using transform_logarithm_slot = expected<pose_reading, refusal> (*)(const transform &tf);

// Lynch & Park, Modern Robotics, sections 3.2.3.3 and 3.3.3.2, eq. 3.92, transcribed literally.
inline rotation_reading book_half_turn(const rotation &r)
{
    const double pi = std::acos(-1.0);

    if(1.0 + r(2, 2) != 0.0)
        return {Eigen::Vector3d(r(0, 2), r(1, 2), 1.0 + r(2, 2)) / std::sqrt(2.0 * (1.0 + r(2, 2))), pi};
    if(1.0 + r(1, 1) != 0.0)
        return {Eigen::Vector3d(r(0, 1), 1.0 + r(1, 1), r(2, 1)) / std::sqrt(2.0 * (1.0 + r(1, 1))), pi};

    return {Eigen::Vector3d(1.0 + r(0, 0), r(1, 0), r(2, 0)) / std::sqrt(2.0 * (1.0 + r(0, 0))), pi};
}

inline rotation_reading book_rotation_reading(const rotation &r)
{
    const double trace = r.trace();

    if(r == rotation::Identity())
        return {Eigen::Vector3d::Zero(), 0.0};
    if(trace == -1.0)
        return book_half_turn(r);

    const double theta          = std::acos((trace - 1.0) / 2.0);
    const Eigen::Matrix3d w_hat = (r - r.transpose()) / (2.0 * std::sin(theta));

    return {Eigen::Vector3d(w_hat(2, 1), w_hat(0, 2), w_hat(1, 0)), theta};
}

inline pose_reading book_pose_reading(const rotation &r, const Eigen::Vector3d &p)
{
    screw_axis s;

    if(r == rotation::Identity())
    {
        const double theta = p.norm();
        s << Eigen::Vector3d::Zero(), p / theta;
        return {s, theta};
    }

    const auto [w, theta]           = book_rotation_reading(r);
    const Eigen::Matrix3d w_hat     = rigid_motion::skew_symmetric(w);
    const Eigen::Matrix3d g_inverse = Eigen::Matrix3d::Identity() / theta - 0.5 * w_hat + (1.0 / theta - 0.5 / std::tan(theta / 2.0)) * w_hat * w_hat;
    s << w, g_inverse * p;

    return {s, theta};
}

inline expected<rotation_reading, refusal> book_logarithm_so3(const rotation &r)
{
    return book_rotation_reading(r);
}

inline expected<pose_reading, refusal> book_logarithm_se3_rp(const rotation &r, const Eigen::Vector3d &p)
{
    return book_pose_reading(r, p);
}

inline expected<pose_reading, refusal> book_logarithm_se3(const transform &tf)
{
    return book_pose_reading(tf.topLeftCorner<3, 3>(), tf.topRightCorner<3, 1>());
}

inline constexpr auto axis_negated = [](auto read)
{
    read.first = -read.first;
    return read;
};
inline constexpr auto angle_doubled = [](auto read)
{
    read.second *= 2.0;
    return read;
};
inline constexpr auto angular_doubled = [](auto read)
{
    read.first.template head<3>() *= 2.0;
    return read;
};
inline constexpr auto axis_undefined = [](auto read)
{
    read.first[0] = std::numeric_limits<double>::quiet_NaN();
    return read;
};

template<const auto &change>
expected<rotation_reading, refusal> changed_so3(const rotation &r)
{
    return moved_or_refused(rigid_motion::matrix_logarithm_so3(r), change);
}

template<const auto &change>
expected<pose_reading, refusal> changed_se3_rp(const rotation &r, const Eigen::Vector3d &p)
{
    return moved_or_refused(rigid_motion::matrix_logarithm_se3_rp(r, p), change);
}

template<const auto &change>
expected<pose_reading, refusal> changed_se3(const transform &tf)
{
    return moved_or_refused(rigid_motion::matrix_logarithm_se3(tf), change);
}

inline expected<rotation_reading, refusal> transposed_so3(const rotation &r)
{
    return rigid_motion::matrix_logarithm_so3(r.transpose());
}

inline expected<pose_reading, refusal> transposed_se3_rp(const rotation &r, const Eigen::Vector3d &p)
{
    return rigid_motion::matrix_logarithm_se3_rp(r.transpose(), p);
}

inline expected<pose_reading, refusal> transposed_se3(const transform &tf)
{
    transform reversed = tf;
    reversed.topLeftCorner<3, 3>().transposeInPlace();

    return rigid_motion::matrix_logarithm_se3(reversed);
}

inline expected<pose_reading, refusal> position_as_linear_half_se3_rp(const rotation &r, const Eigen::Vector3d &p)
{
    const expected<rotation_reading, refusal> read = rigid_motion::matrix_logarithm_so3(r);
    if(!read)
        return unexpected(read.error());

    screw_axis s;
    s << read.value().first, p;

    return pose_reading{s, read.value().second};
}

inline expected<pose_reading, refusal> position_as_linear_half_se3(const transform &tf)
{
    return position_as_linear_half_se3_rp(tf.topLeftCorner<3, 3>(), tf.topRightCorner<3, 1>());
}

inline rigid_motion::capabilities bound_logarithms(rotation_logarithm_slot so3, pose_logarithm_slot se3_rp, transform_logarithm_slot se3)
{
    rigid_motion::capabilities spatial    = rigid_motion::baseline();
    spatial.screw.matrix_logarithm_so3    = so3;
    spatial.screw.matrix_logarithm_se3_rp = se3_rp;
    spatial.screw.matrix_logarithm_se3    = se3;

    return spatial;
}

template<const auto &change>
rigid_motion::capabilities changed_logarithms()
{
    return bound_logarithms(&changed_so3<change>, &changed_se3_rp<change>, &changed_se3<change>);
}

inline rigid_motion::capabilities book_logarithms()
{
    return bound_logarithms(&book_logarithm_so3, &book_logarithm_se3_rp, &book_logarithm_se3);
}

inline rigid_motion::capabilities transposed_logarithms()
{
    return bound_logarithms(&transposed_so3, &transposed_se3_rp, &transposed_se3);
}

// The rotation row keeps the reference, since a rotation's logarithm has no linear half to get wrong.
inline rigid_motion::capabilities position_as_linear_half_logarithms()
{
    return bound_logarithms(&rigid_motion::matrix_logarithm_so3, &position_as_linear_half_se3_rp, &position_as_linear_half_se3);
}

}

#endif
