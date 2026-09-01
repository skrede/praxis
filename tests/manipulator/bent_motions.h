#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_BENT_MOTIONS_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_BENT_MOTIONS_H

#include "bend_kinds.h"

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/baseline/motion.h"
#include "praxis/manipulator/baseline/task_trajectory.h"

#include "praxis/trajectory/trajectory.h"

#include <Eigen/Core>

#include <span>
#include <memory>
#include <utility>

// The bindings whose answer is a configuration a solve reached, or a prepared motion over a run of
// them. Each answers the reference displaced in the one quantity the row comparing it measures.
namespace praxis::fixture {

using namespace manipulator;

inline joint_vector nudged(const joint_vector &answer)
{
    return joint_vector(answer + joint_vector::Constant(answer.size(), bent_by(bent::configuration)));
}

inline expected<joint_vector, refusal> nudged_task_space_pose(const kinematics &solver, const transform &pose, const joint_vector &j0)
{
    const expected<joint_vector, refusal> answer = task_space_pose(solver, pose, j0);
    if(!answer)
        return answer;

    return nudged(answer.value());
}

inline expected<joint_vector, refusal> nudged_task_space_screw(const rigid_motion::screw_ops &screw, const kinematics &solver, const transform &start_pose, const Eigen::Vector3d &w,
                                                               const Eigen::Vector3d &q, double theta_radians, double h, const joint_vector &j0)
{
    const expected<joint_vector, refusal> answer = task_space_screw(screw, solver, start_pose, w, q, theta_radians, h, j0);
    if(!answer)
        return answer;

    return nudged(answer.value());
}

inline expected<joint_vector, refusal> nudged_tool_frame_displace(const kinematics &solver, const transform &start_pose, const Eigen::Vector3d &offset, const rotation &orientation,
                                                                  const joint_vector &j0)
{
    const expected<joint_vector, refusal> answer = tool_frame_displace(solver, start_pose, offset, orientation, j0);
    if(!answer)
        return answer;

    return nudged(answer.value());
}

// A prepared motion answering the reference's, moved in the quantity the row comparing it measures:
// one element of every configuration it names, at every time it is sampled at.
class displaced_motion final : public trajectory::trajectory_generator
{
public:
    explicit displaced_motion(std::unique_ptr<trajectory::trajectory_generator> held)
            : m_held(std::move(held))
    {
    }

    expected<trajectory::trajectory_sample, refusal> sample(double t) const override
    {
        expected<trajectory::trajectory_sample, refusal> taken = m_held->sample(t);
        if(!taken || taken->position.size() == 0)
            return taken;

        taken->position(0) += bent_by(bent::prepared_motion);

        return taken;
    }

    double duration() const override
    {
        return m_held->duration();
    }

private:
    std::unique_ptr<trajectory::trajectory_generator> m_held;
};

inline expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> displaced_task_space_waypoints(const kinematics &solver, std::span<const transform> waypoints,
                                                                                                           const joint_vector &j0, const joint_limits &limits)
{
    expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> motion = task_space_waypoints(solver, waypoints, j0, limits);
    if(!motion || *motion == nullptr)
        return motion;

    return std::unique_ptr<trajectory::trajectory_generator>(std::make_unique<displaced_motion>(std::move(*motion)));
}

}

#endif
