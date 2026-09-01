#include "praxis/manipulator/baseline/robot.h"

namespace praxis::manipulator {

transform tool_pose_from_flange_pose(const transform &flange_pose, const transform &tool_offset)
{
    return flange_pose * tool_offset;
}

transform flange_pose_from_tool_pose(const rigid_motion::frame_ops &frames, const transform &tool_pose, const transform &tool_offset)
{
    return tool_pose * frames.inverse(tool_offset);
}

Eigen::Vector3d position_from_pose(const transform &pose)
{
    return pose.block<3, 1>(0, 3);
}

rotation orientation_from_pose(const transform &pose)
{
    return pose.block<3, 3>(0, 0);
}

// The seam carries no solver parameters into a robot capability, so both resolutions run on the
// contract's own defaults; a caller wanting a different budget reaches the solver directly.
expected<joint_vector, refusal> ik_solve_pose(const kinematics &solver, const rigid_motion::frame_ops &frames, const transform &tool_pose, const joint_vector &j0,
                                              const transform &tool_offset)
{
    return ik_solve_flange_pose(solver, flange_pose_from_tool_pose(frames, tool_pose, tool_offset), j0);
}

expected<joint_vector, refusal> ik_solve_flange_pose(const kinematics &solver, const transform &flange_pose, const joint_vector &j0)
{
    if(j0.size() != static_cast<Eigen::Index>(solver.joint_count()))
        return unexpected(refusal::unsupported_input);

    return solver.ik_solve(flange_pose, j0, solver_parameters());
}

}
