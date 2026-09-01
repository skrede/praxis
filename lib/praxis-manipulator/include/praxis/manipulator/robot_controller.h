#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_CONTROLLER_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_CONTROLLER_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/motion.h"
#include "praxis/manipulator/scene_robot.h"
#include "praxis/manipulator/motion_commands.h"
#include "praxis/manipulator/task_trajectory.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include "praxis/scheduler/task.h"

#include "praxis/trajectory/path.h"
#include "praxis/trajectory/trajectory.h"
#include "praxis/trajectory/time_scaling.h"

#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/types.h"

#include <span>
#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <functional>
#include <filesystem>
#include <string_view>

namespace praxis::manipulator {

struct recording_parameters
{
    bool active;
    std::filesystem::path directory;

    recording_parameters();
};

class trajectory_executor;
struct preview_run;

// The whole control surface the operator windows talk to. Every orientation it reports is a rotation
// matrix; a widget wanting Euler angles converts at the widget, which is also where the
// single-precision boundary is.
class robot_controller
{
public:
    // Marks one command's extent. The first request made of the solver inside one replaces what the
    // command before it left and every later request adds to it; outside one, every request is its
    // own command, which is what bounds what a caller driving this class directly keeps.
    class command_extent
    {
    public:
        explicit command_extent(robot_controller &commanded);
        command_extent(const command_extent &)            = delete;
        command_extent(command_extent &&)                 = delete;
        command_extent &operator=(const command_extent &) = delete;
        command_extent &operator=(command_extent &&)      = delete;
        ~command_extent();

    private:
        robot_controller &m_commanded;
    };

    robot_controller(scene_robot &driven, const motion_ops &injected_motion, const trajectory::path_ops &injected_path, const task_trajectory_ops &injected_task_trajectory,
                     const trajectory::time_scaling_ops &injected_time_scaling, const trajectory::trajectory_ops &injected_joint_trajectory,
                     const rigid_motion::screw_ops &injected_screw, std::function<void()> ask_unload = {}, std::filesystem::path root = {});
    ~robot_controller();

    recording_parameters recording() const;

    // Resolves the folder the parameters name against the root below, prepares it and applies the
    // parameters as one step, answering the folder the rows will be written into. A folder that
    // cannot be prepared leaves the recording holding the settings it already had, so no accepted
    // setting names a folder the run cannot write into.
    expected<std::filesystem::path, refusal> set_recording(const recording_parameters &parameters);

    // Writes the rows a recording in flight has gathered and ends the motion carrying them, so a
    // composition retired part way through a motion leaves the samples taken up to that point rather
    // than nothing at all. The write can fail, which is why the outcome is returned and why this
    // belongs on the strand the arm is serviced on rather than in a destructor.
    expected<void, refusal> conclude_recording();

    std::uint32_t joint_count() const;

    expected<transform, refusal> tool_pose() const;
    expected<Eigen::Vector3d, refusal> tool_position() const;
    expected<rotation, refusal> tool_orientation() const;

    expected<transform, refusal> flange_pose() const;
    expected<Eigen::Vector3d, refusal> flange_position() const;
    expected<rotation, refusal> flange_orientation() const;

    joint_vector joint_positions() const;
    const joint_limits &limits() const;

    bool executing() const;
    bool playback_pending() const;

    // One step of the motion in flight, scaled by the velocity factor. True while a motion remains.
    bool advance_playback(scheduler::step_delta step);

    // Names what refused, then hands the refusal to the one place that decides what becomes of the
    // composition, so every command reports the same way and none of them decides fatality itself.
    // The standing defaults to a fact about this one request, which is what a command is.
    void report_refusal(std::string_view named, refusal reason, refusal_standing standing = refusal_standing::per_request) const;

    double velocity_factor() const;
    void set_velocity_factor(double factor);

    // The choice governs every motion this controller commands, and it is read where a motion is
    // composed rather than where it is played.
    void set_time_scaling(time_scaling_choice chosen);
    time_scaling_choice time_scaling() const;

    // An absent pair leaves a trapezoidal motion at the bounds the motion's own extent gives.
    void set_trapezoid_bounds(std::optional<path_parameter_bounds> held_to);
    std::optional<path_parameter_bounds> trapezoid_bounds() const;

    void preview_tool_frame_jog(const transform &start_pose, const Eigen::Vector3d &offset, const rotation &orientation);
    void preview_task_space_pose(const transform &pose);
    void preview_task_space_screw(const transform &start_pose, const Eigen::Vector3d &w, const Eigen::Vector3d &q, double theta_radians, double pitch);
    void preview_joint_configuration(const joint_vector &positions);

    void task_space_ptp(const transform &pose);
    void task_space_lin(const transform &pose);
    void task_space_screw(const Eigen::Vector3d &w, const Eigen::Vector3d &q, double theta_radians, double pitch);

    void task_space_trajectory(std::span<const transform> poses);
    void joint_space_trajectory(std::span<const joint_vector> positions);

    // Samples the motion the command of the same shape would play, once and whole, and publishes it
    // without commanding anything and without handing the solver a request of its own: the solves a
    // task-space sampling reaches are internal to the drawing, so what a reader sees of the solver is
    // what the last requesting command left. The generator is kept exactly as it was sampled.
    void preview_trajectory(const joint_vector &target);
    void preview_trajectory(const transform &end_pose);
    void preview_trajectory(std::span<const joint_vector> positions);
    void preview_trajectory(std::span<const transform> poses);

    // Samples the run of separate motions between the targets -- one motion per pair, each coming
    // fully to rest at the target it ends at -- and publishes them as one run, so the rate of the
    // path parameter returns to zero at every target instead of carrying through it. The run stands
    // between the targets and not from where the arm is, and the targets are kept, so what the play
    // runs is the sequence that was drawn.
    void preview_in_turn(std::span<const joint_vector> targets);

    // Releases both the sampled motion and the generator it was sampled from.
    void clear_preview();

    // Stands the arm at the kept preview's first sample and then runs the very generator that
    // preview was sampled from, so what plays is what was drawn and plotted rather than a second
    // composition of the same request. The executor takes the generator, so a preview plays once;
    // the sampled run stays published, which is what leaves the commanded path drawn beside the one
    // the tool traverses. A previewed run of separate motions carries targets rather than one
    // generator, and plays as the queued run between those same targets.
    void play_preview();

    std::shared_ptr<const preview_run> preview() const;

    // Plays each target in turn as a motion of its own, so the arm comes fully to rest at every one
    // of them: the next begins where the one before it concluded. A target the composition refuses
    // stops the run there and names it, leaving the rest unplayed.
    void run_in_turn(std::span<const joint_vector> targets);

    // The targets a queued run has not started yet, in the order it will start them. The span stays
    // valid until the run is opened, advanced or cleared, so what leaves the strand is a copy rather
    // than the span.
    std::span<const joint_vector> queued() const;

    void clear_queue();

    // Starts the target a queued run stands at, which is what the conclusion of the motion before it
    // waits on. A run with nothing left commands nothing.
    void start_queued();

    // The tool poses the arm has stood at since the run in flight began, in the order they were
    // reached, spanning every target of a queued run rather than the one in flight. Null where no
    // playback has stepped since the last run was commanded.
    std::shared_ptr<const std::vector<transform>> traversed() const;

    // Asks the solver for the target from each seed in turn, as one command, and moves to whichever
    // of the distinct configurations they answered stands nearest -- in joint space and with each
    // joint difference wrapped to a half turn either way -- the configuration the arm is already at.
    // A seed of the wrong width is declined and the seeds beside it are still solved from.
    void solve_from_seeds(const transform &target, std::span<const joint_vector> seeds);

    // Asks the solver for every configuration that reaches the target in one go, which takes no
    // seed, and moves to whichever of them stands nearest by the same distance.
    void solve_in_closed_form(const transform &target);

    // One sequence per request the most recent requesting command handed the solver and the solver
    // entered, in the order the requests were made. A command that hands the solver no request leaves
    // what stands here alone. The span stays valid until the next request is made, so what leaves the
    // strand is a copy rather than the span.
    std::span<const std::vector<iteration_state>> solves() const;

    // The distinct configurations the most recent requesting command's solves answered, in the order
    // they were first met, with configurations standing within the fold distance of one already held
    // counted as that one again. Emptied where that command's sequences are. The span stays valid
    // until the next request is made, so what leaves the strand is a copy rather than the span.
    std::span<const joint_vector> solutions() const;

    // Which of the configurations above each of the most recent requesting command's starts reached,
    // one entry per sequence solves() carries and in that same order, so the two are read against
    // each other. An entry equal to the number of configurations names a start that entered the
    // solve and reached none of them. The span stays valid until the next request is made, so what
    // leaves the strand is a copy rather than the span.
    std::span<const std::size_t> reached() const;

private:
    motion_ops m_motion;
    trajectory::path_ops m_path;
    task_trajectory_ops m_task_trajectory;
    trajectory::time_scaling_ops m_time_scaling;
    time_scaling_choice m_scaling_choice;
    std::optional<path_parameter_bounds> m_scaling_bounds;
    trajectory::trajectory_ops m_joint_trajectory;
    rigid_motion::screw_ops m_screw;
    double m_velocity;
    std::size_t m_queued_at;
    std::vector<std::size_t> m_reached;
    std::vector<joint_vector> m_queued;
    std::vector<joint_vector> m_solutions;
    // The targets a previewed run of separate motions plays to. The first of them is where the arm
    // is stood before the run begins and so is not among these. Empty where what stands previewed
    // is one motion rather than a run.
    std::vector<joint_vector> m_previewed_targets;
    std::vector<std::vector<iteration_state>> m_solves;
    std::vector<transform> m_traversed;
    std::shared_ptr<const preview_run> m_preview;
    std::shared_ptr<const std::vector<transform>> m_traversed_run;
    std::unique_ptr<trajectory::trajectory_generator> m_previewed;
    std::uint32_t m_open_extents;
    bool m_carried_over;
    scene_robot &m_robot;
    // Every recording folder is resolved against this. An empty one leaves the folder as it was
    // written, which is the resolution a caller that chose no root asked for.
    std::filesystem::path m_root;
    // Absent unless whoever composed this kept a way to say the composition cannot continue. Its
    // target is the composition, which outlives every preset it holds, and this copy dies with the
    // controller, so it can never outlive what it names.
    std::function<void()> m_unload_cb;
    std::unique_ptr<trajectory_executor> m_executor;

    // The one place the accumulator is emptied, announced on the path a request travels: a request
    // outside any extent is its own command, and the first one inside an extent replaces what the
    // command before it left.
    void asking();

    // Keeps the iterates of the last solve this request entered. The tally is the solver's own, read
    // at the request's entry: a request that never reached the solver leaves the tally where it was
    // and leaves no sequence here.
    void keep(std::uint64_t before);

    // Hands back what the solver answered, having kept what the request above keeps.
    expected<joint_vector, refusal> kept(std::uint64_t before, expected<joint_vector, refusal> reached);

    // One start of a multi-start command: the solve, the fold of what it answered into the distinct
    // configurations, and the index naming which of them this start reached.
    void solved_from(const transform &target, const joint_vector &seed);

    // Commands the motion to whichever distinct configuration stands nearest where the arm is, and
    // answers which one that was. Nothing is commanded where none was answered.
    std::optional<std::size_t> run_to_nearest(std::string_view named);

    // The scaling the arm carries, prepared for one composition: the choice and the override are
    // read here, which is why a change to either reaches the next motion and not the one in flight.
    prepared_time_scaling prepared() const;

    // One prepared value per choice, in the enumeration's own order, over the bounds the motion's
    // own extent gives -- which is what lets a preview carry every curve and not only the current
    // one. Empty where the motion refuses either of its endpoints.
    std::vector<prepared_time_scaling> scalings_along(const trajectory::trajectory_generator &motion) const;

    // Samples the motion, publishes what the pass answered and keeps the generator itself.
    void previewed(std::unique_ptr<trajectory::trajectory_generator> motion, std::span<const prepared_time_scaling> scalings);

    // One leg of a previewed run, composed between the pair rather than from where the arm stands
    // and appended to what the legs before it left, so a whole run is sampled without the arm
    // moving through any of it. False where the composition or its sampling refused.
    bool appended_leg(preview_run &whole, const joint_vector &from, const joint_vector &to) const;

    // The one place the traversed run grows. The pose is read after the step, so what it records is
    // where the tool went rather than where it was told to go, and the share is replaced whole.
    void traversed_through();

    // The one place the traversed run is opened. A run is opened where it begins rather than where
    // each of its motions is loaded, so what the tool traversed spans every target of a queued run
    // and not only the one in flight, and the last run's path stands against the next run's
    // commanded one until that next run is actually played.
    void open_traversed_run();

    // One motion a command asked for, which abandons whatever a queued run had left: a learner is
    // never carried on by a motion they did not just ask for.
    void run(std::unique_ptr<trajectory::trajectory_generator> motion);

    // The one place a motion reaches the executor.
    void loaded(std::unique_ptr<trajectory::trajectory_generator> motion);

    void run_to(const joint_vector &target);
    void run_along(const transform &end_pose, task_space_path shape);
};

}

#endif
