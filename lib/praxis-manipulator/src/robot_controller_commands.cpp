#include "praxis/manipulator/robot_controller.h"

#include "robot/solution_placement.h"

#include "fatality.h"
#include "trajectory_preview.h"
#include "trajectory_executor.h"

#include <spdlog/spdlog.h>

#include <span>
#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <optional>
#include <algorithm>
#include <string_view>

namespace praxis::manipulator {

namespace {

// What a start that entered a solve and reached no configuration carries until the command ends,
// where every one of them is written to the number of distinct configurations the command found.
constexpr std::size_t reached_nothing = static_cast<std::size_t>(-1);

// The samples and the curves beside them stand in step, a curve being drawn against the time of the
// sample at its own place, so a curve that has not kept up with the samples is emptied for the whole
// run rather than drawn against the times of the legs before it. Each leg is a path of its own, so
// every curve resets once per leg and none measures the joined run.
void carried_on(std::vector<trajectory::scaling_sample> &curve, std::span<const trajectory::scaling_sample> leg, std::size_t carried)
{
    if(curve.size() != carried || leg.empty())
        curve.clear();
    else
        curve.insert(curve.end(), leg.begin(), leg.end());
}

void append_leg(preview_run &whole, preview_run leg)
{
    const std::size_t carried = whole.samples.size();

    whole.scaling.resize(std::max(whole.scaling.size(), leg.scaling.size()));
    for(std::size_t which = 0; which < whole.scaling.size(); ++which)
        carried_on(whole.scaling[which], which < leg.scaling.size() ? std::span<const trajectory::scaling_sample>(leg.scaling[which]) : std::span<const trajectory::scaling_sample>(),
                   carried);

    carried_on(whole.parameter, leg.parameter, carried);

    for(preview_sample &one : leg.samples)
    {
        one.at += whole.span;
        whole.samples.push_back(std::move(one));
    }

    whole.span += leg.span;
}

}

void robot_controller::report_refusal(std::string_view named, refusal reason, refusal_standing standing) const
{
    if(reason == refusal::not_implemented)
        spdlog::warn("praxis: '{}' is not implemented, so nothing was commanded and the arm is where it was", named);
    else if(standing == refusal_standing::composition_wide)
        spdlog::error("praxis: '{}' refuses at every configuration of this composition, so nothing is commanded through it and everything else the composition was composed for still "
                      "answers",
                      named);
    else
        spdlog::error("praxis: '{}' refused the request, so nothing was commanded and the arm is where it was", named);

    tear_down_if_fatal(named, reason, standing, m_unload_cb);
}

void robot_controller::preview_tool_frame_jog(const transform &start_pose, const Eigen::Vector3d &offset, const rotation &orientation)
{
    if(executing())
        return;

    const std::uint64_t before                    = m_robot.solver().solve_count();
    const expected<joint_vector, refusal> reached = kept(before, m_motion.tool_frame_displace(m_robot.solver(), start_pose, offset, orientation, m_robot.joint_positions()));
    if(reached)
        m_robot.set_joint_positions(*reached);
    else
        report_refusal("motion.tool_frame_displace", reached.error());
}

void robot_controller::preview_task_space_pose(const transform &pose)
{
    if(executing())
        return;

    const std::uint64_t before                    = m_robot.solver().solve_count();
    const expected<joint_vector, refusal> reached = kept(before, m_motion.task_space_pose(m_robot.solver(), pose, m_robot.joint_positions()));
    if(reached)
        m_robot.set_joint_positions(*reached);
    else
        report_refusal("motion.task_space_pose", reached.error());
}

void robot_controller::preview_task_space_screw(const transform &start_pose, const Eigen::Vector3d &w, const Eigen::Vector3d &q, double theta_radians, double pitch)
{
    if(executing())
        return;

    const std::uint64_t before = m_robot.solver().solve_count();
    const expected<joint_vector, refusal> reached =
            kept(before, m_motion.task_space_screw(m_screw, m_robot.solver(), start_pose, w, q, theta_radians, pitch, m_robot.joint_positions()));
    if(reached)
        m_robot.set_joint_positions(*reached);
    else
        report_refusal("motion.task_space_screw", reached.error());
}

void robot_controller::preview_joint_configuration(const joint_vector &positions)
{
    if(executing())
        return;

    if(positions.size() != static_cast<Eigen::Index>(m_robot.joint_count()))
    {
        report_refusal("robot_controller.preview_joint_configuration", refusal::unsupported_input);
        return;
    }

    m_robot.set_joint_positions(positions);
}

void robot_controller::task_space_ptp(const transform &pose)
{
    if(executing())
        return;

    const std::uint64_t before                   = m_robot.solver().solve_count();
    const expected<joint_vector, refusal> target = kept(before, m_motion.task_space_pose(m_robot.solver(), pose, m_robot.joint_positions()));
    if(target)
        run_to(*target);
    else
        report_refusal("motion.task_space_pose", target.error());
}

// The whole set of seeds is one command, so the extent opened here is what makes every solve below
// add to the same sequences, the same distinct configurations and the same indices, and the
// announcement beside it is what empties them: the command answers for what it publishes from the
// moment it asks, so a set of seeds it declines every one of publishes nothing at all.
void robot_controller::solve_from_seeds(const transform &target, std::span<const joint_vector> seeds)
{
    if(executing())
        return;

    if(seeds.empty())
    {
        spdlog::error("praxis: 'ik.inverse_kinematics' was given no seed to start from, so no solve was asked for and the arm is where it was");
        return;
    }

    const command_extent extent(*this);
    asking();

    for(const joint_vector &seed : seeds)
        solved_from(target, seed);

    for(std::size_t &named : m_reached)
        if(named == reached_nothing)
            named = m_solutions.size();

    static_cast<void>(run_to_nearest("ik.inverse_kinematics"));
}

void robot_controller::solve_in_closed_form(const transform &target)
{
    if(executing())
        return;

    const command_extent extent(*this);
    asking();

    const std::uint64_t before                                      = m_robot.solver().solve_count();
    const expected<std::span<const joint_vector>, refusal> answered = m_robot.solver().configurations_reaching(target);
    keep(before);
    if(!answered)
    {
        m_reached.resize(m_solves.size(), m_solutions.size());
        report_refusal("ik.analytic_inverse_kinematics", answered.error());
        return;
    }

    for(const joint_vector &one : *answered)
        static_cast<void>(fold_solution(m_solutions, one));

    const std::optional<std::size_t> nearest = run_to_nearest("ik.analytic_inverse_kinematics");
    m_reached.push_back(nearest.value_or(m_solutions.size()));
}

void robot_controller::task_space_lin(const transform &pose)
{
    if(executing())
        return;
    run_along(pose, m_path.decoupled);
}

void robot_controller::task_space_screw(const Eigen::Vector3d &w, const Eigen::Vector3d &q, double theta_radians, double pitch)
{
    if(executing())
        return;

    const expected<transform, refusal> from = m_robot.tool_pose();
    if(!from)
    {
        report_refusal("fk.forward_kinematics", from.error());
        return;
    }

    const expected<screw_axis, refusal> axis = m_screw.screw_axis_from_point_direction_pitch(q, w, pitch);
    if(!axis)
    {
        report_refusal("rigid_motion.screw_axis_from_point_direction_pitch", axis.error());
        return;
    }

    run_along(m_screw.matrix_exponential_screw(*axis, theta_radians) * *from, m_path.screw);
}

// A waypoint count a factory does not serve is a refused request and not a composition unable to
// answer for itself, so both waypoint commands report and return rather than reaching the fatality
// decision the other commands route their refusals through.
void robot_controller::task_space_trajectory(std::span<const transform> poses)
{
    if(executing())
        return;

    asking();
    auto motion = m_task_trajectory.task_space_waypoints(m_robot.solver(), poses, m_robot.joint_positions(), m_robot.limits());
    if(!motion)
    {
        spdlog::error("praxis: the task-space waypoint factory refused {} waypoints, so no motion is commanded", poses.size());
        return;
    }

    run(std::move(*motion));
}

void robot_controller::joint_space_trajectory(std::span<const joint_vector> positions)
{
    if(executing())
        return;

    auto motion = m_joint_trajectory.joint_space_waypoints(positions, m_robot.joint_positions(), m_robot.limits());
    if(!motion)
    {
        spdlog::error("praxis: the configuration-space waypoint factory refused {} waypoints, so no motion is commanded", positions.size());
        return;
    }

    run(std::move(*motion));
}

// The previews below follow the waypoint commands above rather than the commanded motions: a motion
// a factory will not serve is a refused request and not a composition unable to answer for itself,
// so each reports and returns instead of reaching the fatality decision.
void robot_controller::preview_trajectory(const joint_vector &target)
{
    if(executing())
        return;

    auto motion = joint_space_motion(m_path, prepared(), m_robot.limits(), m_robot.joint_positions(), target);
    if(!motion)
    {
        spdlog::error("praxis: the configuration-space composition refused the previewed target, so no preview stands");
        return;
    }

    const std::vector<prepared_time_scaling> curves = scalings_along(**motion);
    previewed(std::move(*motion), curves);
}

void robot_controller::preview_trajectory(const transform &end_pose)
{
    if(executing())
        return;

    const expected<transform, refusal> start_pose = m_robot.tool_pose();
    if(!start_pose)
    {
        spdlog::error("praxis: the arm's own pose is unavailable, so the previewed task-space motion has nowhere to start and no preview stands");
        return;
    }

    auto motion = task_space_motion(m_motion, prepared(), m_robot.solver(), m_robot.limits(), m_robot.joint_positions(), *start_pose, end_pose, m_path.decoupled);
    if(!motion)
    {
        spdlog::error("praxis: the task-space composition refused the previewed pose, so no preview stands");
        return;
    }

    const std::vector<prepared_time_scaling> curves = scalings_along(**motion);
    previewed(std::move(*motion), curves);
}

void robot_controller::preview_trajectory(std::span<const joint_vector> positions)
{
    if(executing())
        return;

    auto motion = m_joint_trajectory.joint_space_waypoints(positions, m_robot.joint_positions(), m_robot.limits());
    if(!motion)
    {
        spdlog::error("praxis: the configuration-space waypoint factory refused {} waypoints, so no preview stands", positions.size());
        return;
    }

    previewed(std::move(*motion), {});
}

void robot_controller::preview_trajectory(std::span<const transform> poses)
{
    if(executing())
        return;

    auto motion = m_task_trajectory.task_space_waypoints(m_robot.solver(), poses, m_robot.joint_positions(), m_robot.limits());
    if(!motion)
    {
        spdlog::error("praxis: the task-space waypoint factory refused {} waypoints, so no preview stands", poses.size());
        return;
    }

    previewed(std::move(*motion), {});
}

void robot_controller::preview_in_turn(std::span<const joint_vector> targets)
{
    if(executing())
        return;

    if(targets.size() < 2u)
    {
        spdlog::error("praxis: 'robot_controller.preview_in_turn' was given {} targets to run between, so nothing is previewed and whatever stood before it still does", targets.size());
        return;
    }

    preview_run whole{0.0, {}, {}, {}};
    for(std::size_t leg = 1u; leg < targets.size(); ++leg)
        if(!appended_leg(whole, targets[leg - 1u], targets[leg]))
        {
            spdlog::error("praxis: leg {} of the previewed run was refused, so nothing is previewed and whatever stood before it still does", leg);
            return;
        }

    m_preview = std::make_shared<const preview_run>(std::move(whole));
    m_previewed.reset();
    m_previewed_targets.assign(targets.begin() + 1, targets.end());
}

bool robot_controller::appended_leg(preview_run &whole, const joint_vector &from, const joint_vector &to) const
{
    auto motion = joint_space_motion(m_path, prepared(), m_robot.limits(), from, to);
    if(!motion)
        return false;

    const std::vector<prepared_time_scaling> curves = scalings_along(**motion);
    expected<preview_run, refusal> sampled          = sampled_preview(**motion, m_robot, curves);
    if(!sampled)
        return false;

    append_leg(whole, std::move(*sampled));

    return true;
}

void robot_controller::clear_preview()
{
    clear_queue();
    m_preview.reset();
    m_previewed.reset();
    m_previewed_targets.clear();
}

// Nothing is composed here: the generator handed to the executor is the one the sampling pass read,
// so the composition's check that a motion begins where the arm is was made once, when the preview
// was taken, and wherever a scrub has since left the arm is beside the point.
void robot_controller::play_preview()
{
    if(executing())
        return;

    if(m_preview == nullptr || m_preview->samples.empty() || (m_previewed == nullptr && m_previewed_targets.empty()))
    {
        spdlog::error("praxis: no previewed motion stands, so there is nothing to play back and the arm is where it was");
        return;
    }

    // The samples were taken off this arm, so the configuration is the arm's own width and is stood
    // at directly rather than through the command that has a width to decline.
    m_robot.set_joint_positions(m_preview->samples.front().motion.position);
    if(m_previewed != nullptr)
    {
        run(std::move(m_previewed));
        return;
    }

    // The arm stands at the first target now, so the queued run recomposes each leg between the same
    // pair the sampled one was composed between.
    run_in_turn(std::exchange(m_previewed_targets, {}));
}

void robot_controller::run_in_turn(std::span<const joint_vector> targets)
{
    if(executing())
        return;

    if(targets.empty())
    {
        spdlog::error("praxis: 'robot_controller.run_in_turn' was given no target to play, so nothing is commanded and the arm is where it was");
        return;
    }

    m_queued.assign(targets.begin(), targets.end());
    m_queued_at = 0;
    open_traversed_run();
    start_queued();
}

// A target a learner typed is user-driven input, so a composition that refuses one is a declined
// request rather than a composition unable to answer for itself: this reports and returns instead of
// reaching the fatality decision the commanded motions route their refusals through.
void robot_controller::start_queued()
{
    if(m_queued.empty())
        return;

    const joint_vector target = m_queued.front();
    m_queued.erase(m_queued.begin());
    ++m_queued_at;

    auto motion = joint_space_motion(m_path, prepared(), m_robot.limits(), m_robot.joint_positions(), target);
    if(!motion)
    {
        spdlog::error("praxis: the configuration-space composition refused target {} of the queued run, so the run stops there and the {} after it are not played", m_queued_at,
                      m_queued.size());
        clear_queue();
        return;
    }

    loaded(std::move(*motion));
}

std::vector<prepared_time_scaling> robot_controller::scalings_along(const trajectory::trajectory_generator &motion) const
{
    const expected<trajectory::trajectory_sample, refusal> begins = motion.sample(0.0);
    const expected<trajectory::trajectory_sample, refusal> ends   = motion.sample(motion.duration());
    if(!begins || !ends)
        return {};

    const path_parameter_bounds held = m_scaling_bounds.value_or(derived_bounds(m_robot.limits(), begins->position, ends->position));

    return {prepared_time_scaling(m_time_scaling, time_scaling_choice::cubic, held), prepared_time_scaling(m_time_scaling, time_scaling_choice::quintic, held),
            prepared_time_scaling(m_time_scaling, time_scaling_choice::trapezoidal, held)};
}

void robot_controller::previewed(std::unique_ptr<trajectory::trajectory_generator> motion, std::span<const prepared_time_scaling> scalings)
{
    expected<preview_run, refusal> sampled = sampled_preview(*motion, m_robot, scalings);
    if(!sampled)
    {
        spdlog::error("praxis: the previewed motion refused one of its own samples, so no preview stands");
        return;
    }

    m_preview   = std::make_shared<const preview_run>(std::move(*sampled));
    m_previewed = std::move(motion);
}

// The answers are copied off the solver here, where the strand that owns it runs, so what the arm
// publishes is a value of its own rather than a span the next solve would empty.
void robot_controller::solved_from(const transform &target, const joint_vector &seed)
{
    if(seed.size() != static_cast<Eigen::Index>(m_robot.joint_count()))
    {
        spdlog::error("praxis: 'robot_controller.solve_from_seeds' was given a seed of {} joint values for an arm of {} joints, so no solve was started from it and the seeds beside "
                      "it still are",
                      seed.size(), m_robot.joint_count());
        return;
    }

    const std::uint64_t before                    = m_robot.solver().solve_count();
    const expected<joint_vector, refusal> reached = kept(before, m_robot.solver().ik_solve(target, seed, solver_parameters{}));
    if(!reached)
    {
        report_refusal("ik.inverse_kinematics", reached.error());
        m_reached.resize(m_solves.size(), reached_nothing);
        return;
    }

    for(const joint_vector &answered : m_robot.solver().solutions())
        static_cast<void>(fold_solution(m_solutions, answered));

    m_reached.push_back(fold_solution(m_solutions, *reached));
}

std::optional<std::size_t> robot_controller::run_to_nearest(std::string_view named)
{
    if(m_solutions.empty())
        return std::nullopt;

    const std::optional<std::size_t> nearest = nearest_solution(m_solutions, m_robot.joint_positions());
    if(!nearest)
    {
        report_refusal(named, refusal::unsupported_input);
        return std::nullopt;
    }

    run_to(m_solutions[*nearest]);

    return nearest;
}

prepared_time_scaling robot_controller::prepared() const
{
    if(m_scaling_bounds)
        return prepared_time_scaling(m_time_scaling, m_scaling_choice, *m_scaling_bounds);

    return prepared_time_scaling(m_time_scaling, m_scaling_choice);
}

void robot_controller::run(std::unique_ptr<trajectory::trajectory_generator> motion)
{
    clear_queue();
    open_traversed_run();
    loaded(std::move(motion));
}

void robot_controller::loaded(std::unique_ptr<trajectory::trajectory_generator> motion)
{
    m_executor->execute(std::move(motion));
}

void robot_controller::run_to(const joint_vector &target)
{
    const prepared_time_scaling scaled = prepared();
    auto motion                        = joint_space_motion(m_path, scaled, m_robot.limits(), m_robot.joint_positions(), target);
    if(!motion)
    {
        report_refusal("joint_space_motion", motion.error());
        return;
    }

    run(std::move(*motion));
}

void robot_controller::run_along(const transform &end_pose, task_space_path shape)
{
    const expected<transform, refusal> start_pose = m_robot.tool_pose();
    if(!start_pose)
    {
        report_refusal("fk.forward_kinematics", start_pose.error());
        return;
    }

    asking();
    const prepared_time_scaling scaled = prepared();
    auto motion                        = task_space_motion(m_motion, scaled, m_robot.solver(), m_robot.limits(), m_robot.joint_positions(), *start_pose, end_pose, shape);
    if(!motion)
    {
        report_refusal("task_space_motion", motion.error());
        return;
    }

    run(std::move(*motion));
}

}
