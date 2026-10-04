#include "evaluation_cases.h"
#include "evaluation_tables.h"

#include "praxis/trajectory/slots.h"

#include "praxis/evaluation/tolerance.h"
#include "praxis/evaluation/comparators.h"

#include <Eigen/Geometry>

#include <array>
#include <limits>
#include <cstddef>

namespace praxis::trajectory {

namespace {

const path_ops &paths_of(const void *value)
{
    return *static_cast<const path_ops *>(value);
}

evaluation::residual between(const configuration &held, const configuration &against)
{
    return evaluation::element_wise_residual(held, against);
}

evaluation::case_result compare_joint_straight_line(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const joint_path_case example = drawn_joint_path_case(drawn);

    return evaluation::agreed_or_refused(paths_of(first).joint_straight_line(example.start, example.end, example.s),
                                         paths_of(second).joint_straight_line(example.start, example.end, example.s), between, allowed);
}

bool is_a_path_rotation(const Eigen::Matrix3d &m)
{
    return is_approx_equal(Eigen::Matrix3d(m.transpose() * m), Eigen::Matrix3d::Identity(), path_rotation_membership_tolerance) &&
            is_approx_equal(m.determinant(), 1.0, path_rotation_membership_tolerance);
}

// `evaluation::pose_residual`, with the rotation between the two answers held to a group member at
// the path rows' own bound.
evaluation::residual path_pose_residual(const transform &held, const transform &against)
{
    const Eigen::Matrix3d between(held.block<3, 3>(0, 0).transpose() * against.block<3, 3>(0, 0));
    const double moved  = (held.block<3, 1>(0, 3) - against.block<3, 1>(0, 3)).norm();
    const double turned = is_a_path_rotation(between) ? Eigen::AngleAxisd(between).angle() : std::numeric_limits<double>::infinity();

    return evaluation::residual{evaluation::residual_kind::pose, turned, moved};
}

// A small element-wise difference over a matrix that need not be a group member says nothing about
// the pose it stands for, so both task-space rows are judged as poses: a rotation in radians and a
// distance in metres, each against its own bound and never summed.
evaluation::case_result compare_screw(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const pose_path_case example = drawn_pose_path_case(drawn);

    return evaluation::agreed_or_refused(paths_of(first).screw(example.start, example.end, example.s), paths_of(second).screw(example.start, example.end, example.s), path_pose_residual,
                                         allowed);
}

evaluation::case_result compare_decoupled(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const pose_path_case example = drawn_pose_path_case(drawn);

    return evaluation::agreed_or_refused(paths_of(first).decoupled(example.start, example.end, example.s), paths_of(second).decoupled(example.start, example.end, example.s),
                                         path_pose_residual, allowed);
}

constexpr evaluation::residual_kind configuration_kind = evaluation::residual_kind::element_wise;
constexpr evaluation::residual_kind pose_kind          = evaluation::residual_kind::pose;

// The rows are in the enumerator order of path_slot, and each name is spelled exactly as the
// descriptor table spells it. Every slot this aggregate describes is compared here, which is what
// the assertion below the table holds.
constexpr std::array path_table{
        evaluation::slot_evaluation{"path.joint_straight_line", configuration_kind, evaluation::tolerance_of(configuration_kind), &compare_joint_straight_line},
        evaluation::slot_evaluation{"path.screw", pose_kind, evaluation::tolerance_pair{screw_path_tolerance_radians, screw_path_tolerance_metres}, &compare_screw},
        evaluation::slot_evaluation{"path.decoupled", pose_kind, evaluation::tolerance_pair{decoupled_path_tolerance_radians, decoupled_path_tolerance_metres}, &compare_decoupled},
};

static_assert(path_table.size() == static_cast<std::size_t>(path_slot::count));

constexpr evaluation::capability_evaluations<path_ops> evaluated_paths{"trajectory", path_table};

}

const evaluation::capability_evaluations<path_ops> &path_evaluations()
{
    return evaluated_paths;
}

}
