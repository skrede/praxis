#include "evaluation_cases.h"
#include "evaluation_tables.h"

#include "praxis/manipulator/capabilities.h"

#include "praxis/evaluation/comparators.h"

#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/rigid_motion/capabilities.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <numbers>
#include <optional>

namespace praxis::manipulator {

namespace {

constexpr evaluation::residual_kind motion_kind = evaluation::residual_kind::pose;

// How far from the start pose the two rows asked for a displacement are asked to move, as a turn in
// radians and a distance in metres. The displacement is a short one so the pose asked for stands
// near a pose the drawn chain provably reaches; a refusal is then a solve that failed rather than a
// request the chain was never going to answer. The axial travel of a screw is its pitch times its
// turn, so a motion the rotation dominates and one the travel dominates are both drawn.
constexpr double displacement_turn_radians = 0.1;
constexpr double displacement_metres       = 0.1;

const motion_ops &motions_of(const void *value)
{
    return *static_cast<const motion_ops *>(value);
}

// One screw implementation serves both sides of the row that takes one, so the row measures its own
// slot rather than a screw capability neither side is under test for.
const rigid_motion::screw_ops &shared_screw()
{
    static const rigid_motion::screw_ops screw = rigid_motion::baseline().screw;

    return screw;
}

// A number in the closed range either way of `extent`, taken from the one full-turn draw the source
// publishes.
double drawn_within(evaluation::case_source &drawn, double extent)
{
    return extent * drawn.angle_radians() / std::numbers::pi;
}

// A motion answers a configuration and is asked for a pose, so each side's answer is read forward
// through the harness's own map and held against the pose that was asked for rather than against the
// other side's answer. Two answers on different branches of one target agree; two identical answers
// standing at some other pose do not.
evaluation::case_result reaching(const solve_case &solved, const expected<joint_vector, refusal> &held, const expected<joint_vector, refusal> &against, const transform &asked_for,
                                 const evaluation::tolerance_pair &allowed)
{
    if(const std::optional<evaluation::case_result> refused = refusal_outcome(held, against))
        return *refused;

    return reaching_what_was_asked(reached_by(solved, *held), reached_by(solved, *against), asked_for, allowed);
}

evaluation::case_result compare_task_space_pose(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const std::optional<solve_case> solved = drawn_solve(drawn);
    if(!solved)
        return unusable(motion_kind);

    return reaching(*solved, motions_of(first).task_space_pose(solved->solver, solved->reached, solved->seed),
                    motions_of(second).task_space_pose(solved->solver, solved->reached, solved->seed), solved->reached, allowed);
}

// The axis is read in the frame the chain's screws stand in, so the displacement premultiplies the
// start pose, which is how the pose asked for is formed once and handed to both sides. The search
// starts from the configuration the chain stands at the start pose in, which is what a caller asking
// for a displacement from where it is has to hand over.
evaluation::case_result compare_task_space_screw(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const std::optional<solve_case> solved = drawn_solve(drawn);
    if(!solved)
        return unusable(motion_kind);

    const Eigen::Vector3d direction          = drawn.unit_direction();
    const Eigen::Vector3d point              = drawn.position_metres();
    const double turn                        = drawn_within(drawn, displacement_turn_radians);
    const double travel                      = drawn.pitch();
    const expected<screw_axis, refusal> axis = rigid_motion::screw_axis_from_point_direction_pitch(point, direction, travel);
    if(!axis)
        return unusable(motion_kind);

    const transform asked_for = rigid_motion::matrix_exponential_screw(*axis, turn) * solved->reached;

    return reaching(*solved, motions_of(first).task_space_screw(shared_screw(), solved->solver, solved->reached, direction, point, turn, travel, solved->standing),
                    motions_of(second).task_space_screw(shared_screw(), solved->solver, solved->reached, direction, point, turn, travel, solved->standing), asked_for, allowed);
}

// The displacement is read in the tool frame, so it postmultiplies the start pose.
evaluation::case_result compare_tool_frame_displace(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const std::optional<solve_case> solved = drawn_solve(drawn);
    if(!solved)
        return unusable(motion_kind);

    const Eigen::Vector3d offset = drawn.unit_direction() * drawn_within(drawn, displacement_metres);
    const rotation turned(Eigen::AngleAxisd(drawn_within(drawn, displacement_turn_radians), drawn.unit_direction()).toRotationMatrix());
    const transform asked_for = solved->reached * rigid_motion::transformation_matrix_from_rotation_position(turned, offset);

    return reaching(*solved, motions_of(first).tool_frame_displace(solved->solver, solved->reached, offset, turned, solved->standing),
                    motions_of(second).tool_frame_displace(solved->solver, solved->reached, offset, turned, solved->standing), asked_for, allowed);
}

// The rows are in the enumerator order of motion_slot, and each name is spelled exactly as the
// descriptor table spells it.
constexpr std::array motion_table{
        evaluation::slot_evaluation{"motion.task_space_pose", motion_kind, evaluation::tolerance_pair{solved_pose_tolerance_radians, solved_pose_tolerance_metres},
                                    &compare_task_space_pose},
        evaluation::slot_evaluation{"motion.task_space_screw", motion_kind, evaluation::tolerance_pair{solved_pose_tolerance_radians, solved_pose_tolerance_metres},
                                    &compare_task_space_screw},
        evaluation::slot_evaluation{"motion.tool_frame_displace", motion_kind, evaluation::tolerance_pair{solved_pose_tolerance_radians, solved_pose_tolerance_metres},
                                    &compare_tool_frame_displace},
};

static_assert(motion_table.size() == static_cast<std::size_t>(motion_slot::count));

constexpr evaluation::capability_evaluations<motion_ops> evaluated_motions{"manipulator", motion_table};

}

const evaluation::capability_evaluations<motion_ops> &motion_evaluations()
{
    return evaluated_motions;
}

}
