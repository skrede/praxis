#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_SCENE_ROBOT_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_SCENE_ROBOT_H

#include "praxis/manipulator/robot.h"
#include "praxis/manipulator/types.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include "praxis/rigid_motion/types.h"

#include <cstdint>

namespace praxis::manipulator {

// The arm's authoritative state and the one place that calls the robot mathematics, implementing
// none of it: every pose accessor and both inverse-kinematics entry points go out through the
// injected slots. The configuration is held in double here and mirrored into the scene graph
// elsewhere, so no pose is derived from what the renderer holds. Orientation is a rotation matrix
// throughout -- a caller wanting Euler angles converts through the rigid-motion slot that takes an
// axis order.
class scene_robot
{
public:
    // The renderer and the solver each carry a joint count derived from the same description by a
    // different route. One of them is reported and the other is driven, so a disagreement is refused
    // here rather than left for whichever of the two a later reader happens to consult.
    static expected<scene_robot, refusal> compose(kinematics solver, const robot_ops &injected, const rigid_motion::frame_ops &frames, std::uint32_t rendered_joints);

    const kinematics &solver() const;

    std::uint32_t joint_count() const;

    joint_vector joint_positions() const;
    void set_joint_positions(const joint_vector &positions);

    const joint_limits &limits() const;

    const transform &tool_offset() const;
    void set_tool_offset(const transform &offset);
    // False until set_tool_offset has been called, whatever offset it was handed: the offset above reads
    // as the identity both before that call and after one handing it the identity.
    bool tool_offset_known() const;

    // The two derivations a pose is read through. Neither can refuse, so a caller holding a pose it
    // already has takes them directly instead of paying for the forward solve a second time.
    Eigen::Vector3d position_of(const transform &pose) const;
    rotation orientation_of(const transform &pose) const;

    // The frame conversion a commanded pose is carried through, made by whatever the composition
    // bound. The tool offset and the frame operations are this holder's own, so no caller supplies either.
    transform flange_pose_from_tool_pose(const transform &tool_pose) const;
    // The way back, over that same tool offset and bound the same way. Neither of the two can refuse.
    transform tool_pose_from_flange_pose(const transform &flange_pose) const;

    expected<transform, refusal> tool_pose() const;
    expected<Eigen::Vector3d, refusal> tool_position() const;
    expected<rotation, refusal> tool_orientation() const;

    // The same two derivations tool_pose() makes, against a configuration the caller supplies rather
    // than the one held, so a pose at a sampled configuration costs no mutation of the held one. A
    // configuration whose width is not the chain's is refused.
    expected<transform, refusal> tool_pose_at(const joint_vector &at) const;

    expected<transform, refusal> flange_pose() const;
    expected<Eigen::Vector3d, refusal> flange_position() const;
    expected<rotation, refusal> flange_orientation() const;

    expected<joint_vector, refusal> ik_solve_pose(const transform &desired_tool_pose, const joint_vector &j0) const;
    expected<joint_vector, refusal> ik_solve_flange_pose(const transform &desired_flange_pose, const joint_vector &j0) const;

private:
    scene_robot(kinematics solver, const robot_ops &injected, const rigid_motion::frame_ops &frames);

    robot_ops m_robot;
    rigid_motion::frame_ops m_frames;
    transform m_offset;
    bool m_offset_known;
    kinematics m_solver;
    joint_vector m_joints;
    joint_limits m_limits;
};

}

#endif
