#include "praxis/manipulator/scene_robot.h"

#include <spdlog/spdlog.h>

#include <utility>

namespace praxis::manipulator {

scene_robot::scene_robot(kinematics solver, const robot_ops &injected, const rigid_motion::frame_ops &frames)
        : m_robot(injected)
        , m_frames(frames)
        , m_offset(transform::Identity())
        , m_solver(std::move(solver))
        , m_joints(joint_vector::Zero(static_cast<Eigen::Index>(m_solver.joint_count())))
        , m_limits(m_solver.space_chain().limits)
{
}

expected<scene_robot, refusal> scene_robot::compose(kinematics solver, const robot_ops &injected, const rigid_motion::frame_ops &frames, std::uint32_t rendered_joints)
{
    const std::uint32_t solved = solver.joint_count();
    if(rendered_joints != solved)
    {
        spdlog::error("praxis: the rendered arm has {} joints and the solver's chain has {}, so the two cannot be driven as one", rendered_joints, solved);
        return unexpected(refusal::unsupported_input);
    }

    return scene_robot(std::move(solver), injected, frames);
}

const kinematics &scene_robot::solver() const
{
    return m_solver;
}

std::uint32_t scene_robot::joint_count() const
{
    return m_solver.joint_count();
}

joint_vector scene_robot::joint_positions() const
{
    return m_joints;
}

void scene_robot::set_joint_positions(const joint_vector &positions)
{
    m_joints = positions;
}

const joint_limits &scene_robot::limits() const
{
    return m_limits;
}

const transform &scene_robot::tool_offset() const
{
    return m_offset;
}

void scene_robot::set_tool_offset(const transform &offset)
{
    m_offset = offset;
}

Eigen::Vector3d scene_robot::position_of(const transform &pose) const
{
    return m_robot.position_from_pose(pose);
}

rotation scene_robot::orientation_of(const transform &pose) const
{
    return m_robot.orientation_from_pose(pose);
}

expected<transform, refusal> scene_robot::tool_pose() const
{
    const expected<transform, refusal> flange = flange_pose();
    if(!flange)
        return unexpected(flange.error());

    return m_robot.tool_pose_from_flange_pose(*flange, m_offset);
}

expected<Eigen::Vector3d, refusal> scene_robot::tool_position() const
{
    const expected<transform, refusal> pose = tool_pose();
    if(!pose)
        return unexpected(pose.error());

    return position_of(*pose);
}

expected<rotation, refusal> scene_robot::tool_orientation() const
{
    const expected<transform, refusal> pose = tool_pose();
    if(!pose)
        return unexpected(pose.error());

    return orientation_of(*pose);
}

expected<transform, refusal> scene_robot::tool_pose_at(const joint_vector &at) const
{
    if(at.size() != static_cast<Eigen::Index>(joint_count()))
        return unexpected(refusal::unsupported_input);

    const expected<transform, refusal> flange = m_solver.fk_solve(at);
    if(!flange)
        return unexpected(flange.error());

    return m_robot.tool_pose_from_flange_pose(*flange, m_offset);
}

expected<transform, refusal> scene_robot::flange_pose() const
{
    return m_solver.fk_solve(m_joints);
}

expected<Eigen::Vector3d, refusal> scene_robot::flange_position() const
{
    const expected<transform, refusal> pose = flange_pose();
    if(!pose)
        return unexpected(pose.error());

    return position_of(*pose);
}

expected<rotation, refusal> scene_robot::flange_orientation() const
{
    const expected<transform, refusal> pose = flange_pose();
    if(!pose)
        return unexpected(pose.error());

    return orientation_of(*pose);
}

expected<joint_vector, refusal> scene_robot::ik_solve_pose(const transform &desired_tool_pose, const joint_vector &j0) const
{
    return m_robot.ik_solve_pose(m_solver, m_frames, desired_tool_pose, j0, m_offset);
}

expected<joint_vector, refusal> scene_robot::ik_solve_flange_pose(const transform &desired_flange_pose, const joint_vector &j0) const
{
    return m_robot.ik_solve_flange_pose(m_solver, desired_flange_pose, j0);
}

}
