#include "evaluation_tables.h"

#include "praxis/manipulator/evaluation.h"

#include <array>

namespace praxis::manipulator {

std::array<evaluation::evaluation_view, 7> evaluation_views(const capabilities &first, const capabilities &second)
{
    return {evaluation::evaluation_view::of(first.fk, second.fk, forward_kinematics_evaluations()),
            evaluation::evaluation_view::of(first.dk, second.dk, differential_kinematics_evaluations()),
            evaluation::evaluation_view::of(first.ik, second.ik, inverse_kinematics_evaluations()),
            evaluation::evaluation_view::of(first.robot, second.robot, robot_evaluations()),
            evaluation::evaluation_view::of(first.motion, second.motion, motion_evaluations()),
            evaluation::evaluation_view::of(first.modeling, second.modeling, modeling_evaluations()),
            evaluation::evaluation_view::of(first.trajectory, second.trajectory, task_trajectory_evaluations())};
}

}
