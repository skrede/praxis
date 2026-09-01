#include "praxis/manipulator/baseline/motion.h"

#include "praxis/rigid_motion/baseline/frame.h"

#include <Eigen/Core>

namespace praxis::manipulator {

// The seam carries no solver parameters into a motion capability, so the resolution runs on the
// contract's own defaults; a caller wanting a different budget reaches the solver directly.
expected<joint_vector, refusal> task_space_pose(const kinematics &solver, const transform &pose, const joint_vector &j0)
{
    if(j0.size() != static_cast<Eigen::Index>(solver.joint_count()))
        return unexpected(refusal::unsupported_input);

    return solver.ik_solve(pose, j0, solver_parameters());
}

// The axis is expressed in the frame the chain's screws are, so the displacement premultiplies the
// start pose.
expected<joint_vector, refusal> task_space_screw(const rigid_motion::screw_ops &screw, const kinematics &solver, const transform &start_pose, const Eigen::Vector3d &w,
                                                 const Eigen::Vector3d &q, double theta_radians, double h, const joint_vector &j0)
{
    const expected<screw_axis, refusal> axis = screw.screw_axis_from_point_direction_pitch(q, w, h);
    if(!axis)
        return unexpected(axis.error());

    return task_space_pose(solver, screw.matrix_exponential_screw(*axis, theta_radians) * start_pose, j0);
}

// The displacement is read in the tool frame, so it postmultiplies the start pose.
expected<joint_vector, refusal> tool_frame_displace(const kinematics &solver, const transform &start_pose, const Eigen::Vector3d &offset, const rotation &orientation,
                                                    const joint_vector &j0)
{
    return task_space_pose(solver, start_pose * rigid_motion::transformation_matrix_from_rotation_position(orientation, offset), j0);
}

}
