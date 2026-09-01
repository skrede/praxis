#include "evaluation_cases.h"
#include "evaluation_tables.h"

#include "praxis/manipulator/capabilities.h"

#include "praxis/evaluation/comparators.h"

#include "praxis/rigid_motion/capabilities.h"

#include <array>
#include <optional>

namespace praxis::manipulator {

namespace {

const robot_ops &robot_of(const void *value)
{
    return *static_cast<const robot_ops *>(value);
}

// One frame implementation serves both sides of every row that takes one, so a row measures its own
// slot rather than a frame capability neither side is under test for.
const rigid_motion::frame_ops &shared_frames()
{
    static const rigid_motion::frame_ops frames = rigid_motion::baseline().frame;

    return frames;
}

// The flange the answer reaches carried out to the tool the row was asked for, along the offset both
// sides were handed.
std::optional<transform> reached_at_the_tool(const solve_case &solved, const joint_vector &answer, const transform &offset)
{
    const std::optional<transform> flange = reached_by(solved, answer);
    if(!flange)
        return std::nullopt;

    return transform(*flange * offset);
}

// The rows are in the enumerator order of robot_slot, and each name is spelled exactly as the
// descriptor table spells it. A rotation is compared by the geodesic angle between the two and never
// element-wise, because a small element-wise number over a matrix that is not a group member says
// nothing about the rotation it was meant to be.
constexpr std::array robot_table{
        evaluation::slot_evaluation{"robot.tool_pose_from_flange_pose", evaluation::residual_kind::pose, evaluation::tolerance_of(evaluation::residual_kind::pose),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const transform flange = drawn.transform_member();
                                        const transform offset = drawn.transform_member();

                                        return judged(evaluation::pose_residual(robot_of(first).tool_pose_from_flange_pose(flange, offset),
                                                                                robot_of(second).tool_pose_from_flange_pose(flange, offset)),
                                                      allowed);
                                    }},
        evaluation::slot_evaluation{"robot.flange_pose_from_tool_pose", evaluation::residual_kind::pose, evaluation::tolerance_of(evaluation::residual_kind::pose),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const transform tool   = drawn.transform_member();
                                        const transform offset = drawn.transform_member();

                                        return judged(evaluation::pose_residual(robot_of(first).flange_pose_from_tool_pose(shared_frames(), tool, offset),
                                                                                robot_of(second).flange_pose_from_tool_pose(shared_frames(), tool, offset)),
                                                      allowed);
                                    }},
        evaluation::slot_evaluation{"robot.position_from_pose", evaluation::residual_kind::element_wise, evaluation::tolerance_of(evaluation::residual_kind::element_wise),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const transform pose = drawn.transform_member();

                                        return judged(evaluation::element_wise_residual(robot_of(first).position_from_pose(pose), robot_of(second).position_from_pose(pose)), allowed);
                                    }},
        evaluation::slot_evaluation{"robot.orientation_from_pose", evaluation::residual_kind::geodesic, evaluation::tolerance_of(evaluation::residual_kind::geodesic),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const transform pose = drawn.transform_member();

                                        return judged(evaluation::geodesic_residual(robot_of(first).orientation_from_pose(pose), robot_of(second).orientation_from_pose(pose)), allowed);
                                    }},
        evaluation::slot_evaluation{"robot.ik_solve_pose", evaluation::residual_kind::pose, evaluation::tolerance_pair{solved_pose_tolerance_radians, solved_pose_tolerance_metres},
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const std::optional<solve_case> solved = drawn_solve(drawn);
                                        if(!solved)
                                            return unusable(evaluation::residual_kind::pose);

                                        const transform offset = drawn.transform_member();
                                        const transform tool   = solved->reached * offset;

                                        const expected<joint_vector, refusal> held    = robot_of(first).ik_solve_pose(solved->solver, shared_frames(), tool, solved->seed, offset);
                                        const expected<joint_vector, refusal> against = robot_of(second).ik_solve_pose(solved->solver, shared_frames(), tool, solved->seed, offset);
                                        if(const std::optional<evaluation::case_result> refused = refusal_outcome(held, against))
                                            return *refused;

                                        return reaching_what_was_asked(reached_at_the_tool(*solved, *held, offset), reached_at_the_tool(*solved, *against, offset), tool, allowed);
                                    }},
        evaluation::slot_evaluation{"robot.ik_solve_flange_pose", evaluation::residual_kind::pose,
                                    evaluation::tolerance_pair{solved_pose_tolerance_radians, solved_pose_tolerance_metres},
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const std::optional<solve_case> solved = drawn_solve(drawn);
                                        if(!solved)
                                            return unusable(evaluation::residual_kind::pose);

                                        const expected<joint_vector, refusal> held    = robot_of(first).ik_solve_flange_pose(solved->solver, solved->reached, solved->seed);
                                        const expected<joint_vector, refusal> against = robot_of(second).ik_solve_flange_pose(solved->solver, solved->reached, solved->seed);
                                        if(const std::optional<evaluation::case_result> refused = refusal_outcome(held, against))
                                            return *refused;

                                        return reaching_what_was_asked(reached_by(*solved, *held), reached_by(*solved, *against), solved->reached, allowed);
                                    }},
};

static_assert(robot_table.size() == static_cast<std::size_t>(robot_slot::count));

constexpr evaluation::capability_evaluations<robot_ops> evaluated_robots{"manipulator", robot_table};

}

const evaluation::capability_evaluations<robot_ops> &robot_evaluations()
{
    return evaluated_robots;
}

}
