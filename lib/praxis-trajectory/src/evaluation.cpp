#include "evaluation_tables.h"

#include "praxis/trajectory/evaluation.h"

#include <array>

namespace praxis::trajectory {

std::array<evaluation::evaluation_view, 4> evaluation_views(const capabilities &first, const capabilities &second)
{
    return {evaluation::evaluation_view::of(first.time_scaling, second.time_scaling, time_scaling_evaluations()),
            evaluation::evaluation_view::of(first.path, second.path, path_evaluations()),
            evaluation::evaluation_view::of(first.pose_trajectory, second.pose_trajectory, pose_trajectory_evaluations()),
            evaluation::evaluation_view::of(first.trajectory, second.trajectory, trajectory_evaluations())};
}

}
