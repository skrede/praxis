#include "evaluation_cases.h"
#include "evaluation_tables.h"

#include "praxis/trajectory/slots.h"

#include "praxis/evaluation/comparators.h"

#include <array>
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

// A small element-wise difference over a matrix that need not be a group member says nothing about
// the pose it stands for, so both task-space rows are judged as poses: a rotation in radians and a
// distance in metres, each against its own bound and never summed.
evaluation::case_result compare_screw(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const pose_path_case example = drawn_pose_path_case(drawn);

    return evaluation::agreed_or_refused(paths_of(first).screw(example.start, example.end, example.s), paths_of(second).screw(example.start, example.end, example.s),
                                         evaluation::pose_residual, allowed);
}

evaluation::case_result compare_decoupled(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const pose_path_case example = drawn_pose_path_case(drawn);

    return evaluation::agreed_or_refused(paths_of(first).decoupled(example.start, example.end, example.s), paths_of(second).decoupled(example.start, example.end, example.s),
                                         evaluation::pose_residual, allowed);
}

constexpr evaluation::residual_kind configuration_kind = evaluation::residual_kind::element_wise;
constexpr evaluation::residual_kind pose_kind          = evaluation::residual_kind::pose;

// The rows are in the enumerator order of path_slot, and each name is spelled exactly as the
// descriptor table spells it. Every slot this aggregate describes is compared here, which is what
// the assertion below the table holds.
constexpr std::array path_table{
        evaluation::slot_evaluation{"path.joint_straight_line", configuration_kind, evaluation::tolerance_of(configuration_kind), &compare_joint_straight_line},
        evaluation::slot_evaluation{"path.screw", pose_kind, evaluation::tolerance_of(pose_kind), &compare_screw},
        evaluation::slot_evaluation{"path.decoupled", pose_kind, evaluation::tolerance_of(pose_kind), &compare_decoupled},
};

static_assert(path_table.size() == static_cast<std::size_t>(path_slot::count));

constexpr evaluation::capability_evaluations<path_ops> evaluated_paths{"trajectory", path_table};

}

const evaluation::capability_evaluations<path_ops> &path_evaluations()
{
    return evaluated_paths;
}

}
