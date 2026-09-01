#include "praxis/manipulator/robot_controller.h"

#include "trajectory_executor.h"

#include "praxis/config/store.h"

#include <spdlog/spdlog.h>

#include <span>
#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <optional>
#include <functional>
#include <filesystem>
#include <system_error>

namespace praxis::manipulator {

namespace {

// The most points a traversed run stands at before its density is halved.
constexpr std::size_t most_traversed_points = 4096;

// Both ends are kept, so the run still spans the whole motion and only the detail between its ends
// falls away.
void halve_density(std::vector<transform> &run)
{
    std::size_t kept = 0;
    for(std::size_t at = (run.size() + 1) % 2; at < run.size(); at += 2)
        run[kept++] = run[at];

    run.resize(kept);
}

}

// The path is empty by default.
recording_parameters::recording_parameters()
        : active(false)
        , directory()
{
}

robot_controller::robot_controller(scene_robot &driven, const motion_ops &injected_motion, const trajectory::path_ops &injected_path,
                                   const task_trajectory_ops &injected_task_trajectory, const trajectory::time_scaling_ops &injected_time_scaling,
                                   const trajectory::trajectory_ops &injected_joint_trajectory, const rigid_motion::screw_ops &injected_screw, std::function<void()> ask_unload,
                                   std::filesystem::path root)
        : m_motion(injected_motion)
        , m_path(injected_path)
        , m_task_trajectory(injected_task_trajectory)
        , m_time_scaling(injected_time_scaling)
        , m_scaling_choice(time_scaling_choice::quintic)
        , m_scaling_bounds(std::nullopt)
        , m_joint_trajectory(injected_joint_trajectory)
        , m_screw(injected_screw)
        , m_velocity(0.3)
        , m_queued_at(0)
        , m_reached()
        , m_queued()
        , m_solutions()
        , m_previewed_targets()
        , m_solves()
        , m_traversed()
        , m_preview()
        , m_traversed_run()
        , m_previewed()
        , m_open_extents(0)
        , m_carried_over(false)
        , m_robot(driven)
        , m_root(std::move(root))
        , m_unload_cb(std::move(ask_unload))
        , m_executor(std::make_unique<trajectory_executor>(driven))
{
}

robot_controller::~robot_controller() = default;

robot_controller::command_extent::command_extent(robot_controller &commanded)
        : m_commanded(commanded)
{
    if(m_commanded.m_open_extents++ == 0)
        m_commanded.m_carried_over = true;
}

robot_controller::command_extent::~command_extent()
{
    --m_commanded.m_open_extents;
}

recording_parameters robot_controller::recording() const
{
    return m_executor->recording();
}

expected<std::filesystem::path, refusal> robot_controller::set_recording(const recording_parameters &parameters)
{
    const config::location where = config::resolve(parameters.directory, m_root);

    std::error_code failure;
    if(!where.resolved.empty())
        create_directories(where.resolved, failure);
    if(failure)
    {
        spdlog::error("praxis: unable to prepare recording folder '{}': {}; the settings are not applied and the recording keeps the ones it had", where.resolved.string(),
                      failure.message());
        return unexpected(refusal::unsupported_input);
    }

    recording_parameters applied = parameters;
    applied.directory            = where.resolved;
    m_executor->set_recording(std::move(applied));

    return where.resolved;
}

expected<void, refusal> robot_controller::conclude_recording()
{
    const expected<std::filesystem::path, refusal> written = m_executor->stop();
    if(!written)
        return unexpected(written.error());

    if(!written->empty())
        spdlog::info("praxis: the trajectory recording in '{}' is concluded and holds the rows it had", written->string());

    return {};
}

std::uint32_t robot_controller::joint_count() const
{
    return m_robot.joint_count();
}

expected<transform, refusal> robot_controller::tool_pose() const
{
    return m_robot.tool_pose();
}

expected<Eigen::Vector3d, refusal> robot_controller::tool_position() const
{
    return m_robot.tool_position();
}

expected<rotation, refusal> robot_controller::tool_orientation() const
{
    return m_robot.tool_orientation();
}

expected<transform, refusal> robot_controller::flange_pose() const
{
    return m_robot.flange_pose();
}

expected<Eigen::Vector3d, refusal> robot_controller::flange_position() const
{
    return m_robot.flange_position();
}

expected<rotation, refusal> robot_controller::flange_orientation() const
{
    return m_robot.flange_orientation();
}

joint_vector robot_controller::joint_positions() const
{
    return m_robot.joint_positions();
}

const joint_limits &robot_controller::limits() const
{
    return m_robot.limits();
}

bool robot_controller::executing() const
{
    return m_executor->executing();
}

bool robot_controller::playback_pending() const
{
    return m_executor->pending();
}

bool robot_controller::advance_playback(scheduler::step_delta step)
{
    const expected<void, refusal> stepped = m_executor->advance(step, m_velocity);
    if(!stepped)
        report_refusal("the motion being played", stepped.error());

    traversed_through();

    return m_executor->executing();
}

// The share is replaced after every step and never appended to through a live one, so a reader never
// sees the run mid-growth and the path it draws ends where the arm stands.
void robot_controller::traversed_through()
{
    const expected<transform, refusal> reached = m_robot.tool_pose();
    if(!reached)
        return;

    m_traversed.push_back(*reached);
    if(m_traversed.size() > most_traversed_points)
        halve_density(m_traversed);

    m_traversed_run = std::make_shared<const std::vector<transform>>(m_traversed);
}

void robot_controller::open_traversed_run()
{
    m_traversed.clear();
    m_traversed_run.reset();
}

std::shared_ptr<const preview_run> robot_controller::preview() const
{
    return m_preview;
}

std::span<const joint_vector> robot_controller::queued() const
{
    return m_queued;
}

void robot_controller::clear_queue()
{
    m_queued.clear();
    m_queued_at = 0;
}

std::shared_ptr<const std::vector<transform>> robot_controller::traversed() const
{
    return m_traversed_run;
}

double robot_controller::velocity_factor() const
{
    return m_velocity;
}

void robot_controller::set_velocity_factor(double factor)
{
    m_velocity = factor;
}

void robot_controller::set_time_scaling(time_scaling_choice chosen)
{
    m_scaling_choice = chosen;
}

time_scaling_choice robot_controller::time_scaling() const
{
    return m_scaling_choice;
}

void robot_controller::set_trapezoid_bounds(std::optional<path_parameter_bounds> held_to)
{
    m_scaling_bounds = held_to;
}

std::optional<path_parameter_bounds> robot_controller::trapezoid_bounds() const
{
    return m_scaling_bounds;
}

std::span<const std::vector<iteration_state>> robot_controller::solves() const
{
    return m_solves;
}

std::span<const joint_vector> robot_controller::solutions() const
{
    return m_solutions;
}

std::span<const std::size_t> robot_controller::reached() const
{
    return m_reached;
}

void robot_controller::asking()
{
    if(m_open_extents == 0 || std::exchange(m_carried_over, false))
    {
        m_solves.clear();
        m_solutions.clear();
        m_reached.clear();
    }
}

void robot_controller::keep(std::uint64_t before)
{
    asking();

    if(m_robot.solver().solve_count() == before)
        return;

    const std::span<const iteration_state> recorded = m_robot.solver().iterations();
    m_solves.emplace_back(recorded.begin(), recorded.end());
}

expected<joint_vector, refusal> robot_controller::kept(std::uint64_t before, expected<joint_vector, refusal> reached)
{
    keep(before);

    return reached;
}

}
