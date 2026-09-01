#include "fixtures.h"
#include "captured_log.h"
#include "trajectory_preview.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/robot_controller.h"

#include "praxis/evaluation/tolerance.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <span>
#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

// The density a previewed motion is drawn at, written here so that changing it has to be done twice
// on purpose rather than once by accident.
constexpr std::size_t previewed_samples = 385;

// How far the parameter a motion realizes may stand from the analytic scaling that composed it, in
// the value and in either derivative. The composed motion's own path derivatives are numerical, and
// the second of them is what the whole of this leaves room for.
constexpr double reproduced_within = 1.0e-6;

// The rate of the path parameter at an instant the motion is at rest, and at one it is moving
// through. The two are apart by seven orders, so no run is read as both.
constexpr double at_rest_below = 1.0e-9;
constexpr double moving_above  = 1.0e-2;

constexpr step_delta stepped{seconds{0.01}};
constexpr std::uint32_t most_steps = 100000;

time_point dictated{};

time_point reading()
{
    return dictated;
}

clock_source dictating()
{
    dictated = time_point{};

    return clock_source{&reading};
}

// A composition rather than the inert stand-in, so the pose at one sampled configuration differs
// from the pose at the next and a preview carrying one pose throughout would fail.
transform offset_tool_pose(const transform &pose, const transform &offset)
{
    return transform(pose * offset);
}

// The answer is the desired pose's own translation, so a resolved path reaches a configuration the
// case can name, and each entry into the solver leaves one iterate behind.
expected<void, refusal> recording_inverse_kinematics(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &, const transform &desired,
                                                     const joint_vector &j0, const solver_parameters &, ik_result &answer)
{
    answer.iterations.push_back(iteration_state{j0, 0.5, 0.25, 0.125, 7u});
    answer.solutions.push_back(configuration(desired(0, 3), desired(1, 3)));

    return {};
}

expected<joint_vector, refusal> solving_task_space_pose(const kinematics &solver, const transform &pose, const joint_vector &j0)
{
    return solver.ik_solve(pose, j0, solver_parameters{});
}

transform reachable(double x, double y)
{
    transform pose = transform::Identity();
    pose(0, 3)     = x;
    pose(1, 3)     = y;

    return pose;
}

struct previewed_arm
{
    std::shared_ptr<scene_robot> robot;
    std::unique_ptr<robot_controller> control;
};

previewed_arm arm_at(const joint_vector &start)
{
    auto robot = std::make_shared<scene_robot>(
            scene_robot::compose(kinematics::compose(sliding_chain(), forward_kinematics_ops{.forward_kinematics = &sliding_forward_kinematics}, differential_kinematics_ops{},
                                                     inverse_kinematics_ops{&recording_inverse_kinematics}, rigid_motion::baseline().screw, rigid_motion::baseline().frame)
                                         .value(),
                                 robot_ops{.tool_pose_from_flange_pose = &offset_tool_pose}, rigid_motion::baseline().frame, 2u)
                    .value());
    robot->set_joint_positions(start);

    auto control = std::make_unique<robot_controller>(*robot, motion_ops{.task_space_pose = &solving_task_space_pose}, trajectory::baseline().path, task_trajectory_ops{},
                                                      trajectory::baseline().time_scaling, trajectory::baseline().trajectory, rigid_motion::screw_ops{});

    return previewed_arm{std::move(robot), std::move(control)};
}

struct arm_pipe
{
    arm_reader seen;
    std::shared_ptr<owned_arm> owned;
    std::shared_ptr<scene_robot> robot;
    std::shared_ptr<robot_controller> control;
};

arm_pipe pipe(praxis::scheduler::scheduler &loop)
{
    const strand work                         = *loop.make_strand();
    previewed_arm arm                         = arm_at(configuration(0.0, 0.0));
    std::shared_ptr<robot_controller> control = std::move(arm.control);
    auto published                            = std::make_shared<arm_publisher>();

    return arm_pipe{published->reader(), std::make_shared<owned_arm>(work, work, arm.robot, control, published), arm.robot, control};
}

std::uint32_t steps_of(robot_controller &control)
{
    std::uint32_t taken = 0;
    for(; taken < most_steps && control.executing(); ++taken)
        static_cast<void>(control.advance_playback(stepped));

    return taken;
}

bool played_out(robot_controller &control)
{
    static_cast<void>(steps_of(control));

    return !control.executing();
}

// The time one step carries into the motion's own clock, which is what a step count is read against.
double played_per_step(const robot_controller &control)
{
    return control.velocity_factor() * stepped.value.count();
}

// Which sample of the run stands nearest the configuration given, which is how a case names the
// instant a run reaches a waypoint whose knot time the spline keeps to itself.
std::size_t stood_nearest(const preview_run &run, const joint_vector &waypoint)
{
    std::size_t nearest = 0;
    double apart        = (run.samples.front().motion.position - waypoint).norm();
    for(std::size_t at = 1u; at < run.samples.size(); ++at)
        if(const double here = (run.samples[at].motion.position - waypoint).norm(); here < apart)
        {
            apart   = here;
            nearest = at;
        }

    return nearest;
}

// True where some pose the run recorded is the one given, which is what a run passing through a
// target reads as: the arm stands exactly at each target it comes to rest at.
bool stood_at(const std::vector<transform> &run, const transform &pose)
{
    for(const transform &reached : run)
        if(is_approx_equal(reached, pose, 1.0e-9))
            return true;

    return false;
}

// A queued run is carried on by the arm's own tick, so a case over one drives the aggregate: the
// periodic playback the arm registers, and a dictated reading raised one step at a time.
bool ticked_out(praxis::scheduler::scheduler &loop, const robot_controller &control)
{
    for(std::uint32_t taken = 0; taken < most_steps && control.executing(); ++taken)
    {
        dictated += std::chrono::duration_cast<time_point::duration>(stepped.value);
        if(!loop.drain().has_value())
            return false;
    }

    return !control.executing();
}

}

TEST_CASE("a previewed configuration-space motion stands at its start and ends at its target", "[manipulator][trajectory]")
{
    const previewed_arm arm = arm_at(configuration(0.0, 0.0));

    arm.control->preview_trajectory(configuration(0.4, -0.2));

    const std::shared_ptr<const preview_run> run = arm.control->preview();

    REQUIRE(run != nullptr);
    REQUIRE(run->samples.size() == previewed_samples);
    CHECK(run->span > 0.0);
    CHECK(is_approx_equal(run->samples.front().motion.position, configuration(0.0, 0.0), 1.0e-9));
    CHECK(is_approx_equal(run->samples.back().motion.position, configuration(0.4, -0.2), 1.0e-9));
    CHECK(run->samples.front().at == 0.0);
    CHECK(is_approx_equal(run->samples.back().at, run->span, 1.0e-9));
}

// A pose taken at anything but the sample's own configuration would draw a path that the plot beside
// it and the scrub through it both disagree with.
TEST_CASE("every previewed sample carries the tool pose at its own configuration", "[manipulator][trajectory]")
{
    const previewed_arm arm = arm_at(configuration(0.0, 0.0));

    arm.control->preview_trajectory(configuration(0.4, -0.2));

    const std::shared_ptr<const preview_run> run = arm.control->preview();

    REQUIRE(run != nullptr);
    for(const preview_sample &one : run->samples)
    {
        REQUIRE(one.tool_pose.has_value());
        REQUIRE(is_approx_equal(*one.tool_pose, arm.robot->tool_pose_at(one.motion.position).value(), 1.0e-9));
    }

    CHECK_FALSE(is_approx_equal(*run->samples.front().tool_pose, *run->samples.back().tool_pose, 1.0e-9));
}

TEST_CASE("a previewed motion driven by a chosen scaling carries one curve per scaling", "[manipulator][trajectory]")
{
    const previewed_arm arm = arm_at(configuration(0.0, 0.0));

    arm.control->preview_trajectory(configuration(0.4, -0.2));

    const std::shared_ptr<const preview_run> run = arm.control->preview();

    REQUIRE(run != nullptr);
    REQUIRE(run->scaling.size() == 3u);
    CHECK(run->scaling[0].size() == run->samples.size());
    CHECK(run->scaling[1].size() == run->samples.size());
    CHECK(run->scaling[2].size() == run->samples.size());

    const std::size_t quarter                    = (previewed_samples - 1) / 4;
    const double t                               = run->samples[quarter].at;
    const trajectory::time_scaling_ops reference = trajectory::baseline().time_scaling;

    CHECK(is_approx_equal(run->scaling[0][quarter].s, reference.cubic(t, run->span).value().s, 1.0e-9));
    CHECK(is_approx_equal(run->scaling[1][quarter].s, reference.quintic(t, run->span).value().s, 1.0e-9));
    CHECK(run->scaling[0][quarter].s != run->scaling[1][quarter].s);
    CHECK(run->scaling[2][quarter].s != run->scaling[1][quarter].s);
}

// The whole distinction between running a list of rows as separate motions and running it as one:
// the rate of the path parameter returns to zero at every row rather than carrying through it, and
// the run stands between the first row and the last rather than starting where the arm happens to be.
TEST_CASE("a previewed run of separate motions stands at every row and comes to rest at each", "[manipulator][trajectory]")
{
    const previewed_arm arm = arm_at(configuration(0.0, 0.0));
    const std::vector<joint_vector> rows{configuration(0.3, 0.1), configuration(-0.2, 0.4), configuration(0.5, -0.3)};

    arm.control->preview_in_turn(rows);

    const std::shared_ptr<const preview_run> run = arm.control->preview();

    REQUIRE(run != nullptr);
    REQUIRE(run->samples.size() == 2u * previewed_samples);
    CHECK(is_approx_equal(run->samples.front().motion.position, rows.front(), 1.0e-9));
    CHECK(is_approx_equal(run->samples.back().motion.position, rows.back(), 1.0e-9));
    CHECK(is_approx_equal(run->samples[previewed_samples - 1u].motion.position, rows[1], 1.0e-9));
    CHECK(is_approx_equal(arm.robot->joint_positions(), configuration(0.0, 0.0), 1.0e-12));

    REQUIRE(run->scaling.size() == 3u);
    CHECK(run->scaling.front().size() == run->samples.size());
    CHECK(run->scaling.front().front().ds < 1.0e-9);
    CHECK(run->scaling.front()[previewed_samples - 1u].ds < 1.0e-9);
    CHECK(run->scaling.front()[previewed_samples / 2u].ds > 1.0e-3);
    CHECK(run->scaling.front().back().ds < 1.0e-9);

    // The legs meet at one instant and the times run on across them, so the plot is one axis.
    CHECK(is_approx_equal(run->samples[previewed_samples].at, run->samples[previewed_samples - 1u].at, 1.0e-9));
    CHECK(is_approx_equal(run->samples.back().at, run->span, 1.0e-9));
}

TEST_CASE("a previewed run of separate motions plays as that same run, standing at every row", "[manipulator][trajectory]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const arm_pipe arm                      = pipe(loop);
    const std::weak_ptr<owned_arm> observer = arm.owned;
    const std::vector<joint_vector> rows{configuration(0.3, 0.1), configuration(-0.2, 0.4), configuration(0.5, -0.3)};

    command(observer, [](robot_controller &control, scene_robot &) { control.set_velocity_factor(1.0); });
    command(observer, [rows](robot_controller &control, scene_robot &) { control.preview_in_turn(rows); });
    command(observer, [](robot_controller &control, scene_robot &) { control.play_preview(); });
    REQUIRE(loop.drain().has_value());

    const std::shared_ptr<const preview_run> run = arm.control->preview();

    REQUIRE(run != nullptr);
    REQUIRE(arm.control->executing());
    CHECK(arm.control->queued().size() == 1u);
    CHECK(is_approx_equal(arm.robot->joint_positions(), rows.front(), 1.0e-12));

    std::vector<joint_vector> stood;
    std::size_t left = arm.control->queued().size();
    for(std::uint32_t taken = 0; taken < most_steps && arm.control->executing(); ++taken)
    {
        dictated += std::chrono::duration_cast<time_point::duration>(stepped.value);
        REQUIRE(loop.drain().has_value());
        if(arm.control->queued().size() != left)
        {
            stood.push_back(arm.robot->joint_positions());
            left = arm.control->queued().size();
        }
    }

    REQUIRE(stood.size() == 1u);
    CHECK(is_approx_equal(stood.front(), rows[1], 1.0e-12));
    CHECK(is_approx_equal(arm.robot->joint_positions(), rows.back(), 1.0e-12));

    // The play consumed the run, and the samples it was drawn from stand where they were.
    CHECK(arm.control->preview().get() == run.get());

    const std::string reported = praxis::tests::reported_by([&] { arm.control->play_preview(); });

    CHECK_THAT(reported, Catch::Matchers::ContainsSubstring("play"));
    CHECK_FALSE(arm.control->executing());
}

// A via-point motion's timing is its own polynomial, so there is no separately chosen scaling to draw
// beside it and the absence is stated once rather than once per sample.
TEST_CASE("a previewed via-point motion carries no scaling curve at all", "[manipulator][trajectory]")
{
    const previewed_arm arm                   = arm_at(configuration(0.0, 0.0));
    const std::vector<joint_vector> waypoints = {configuration(0.2, 0.3), configuration(0.4, -0.2)};

    arm.control->preview_trajectory(std::span<const joint_vector>(waypoints));

    const std::shared_ptr<const preview_run> run = arm.control->preview();

    REQUIRE(run != nullptr);
    CHECK(run->scaling.empty());
    CHECK(run->samples.size() == previewed_samples);
    CHECK(run->parameter.size() == previewed_samples);
}

TEST_CASE("the realized parameter runs from zero to one over the samples and never turns back", "[manipulator][trajectory]")
{
    const previewed_arm arm = arm_at(configuration(0.0, 0.0));

    arm.control->preview_trajectory(configuration(0.4, -0.2));

    const std::shared_ptr<const preview_run> run = arm.control->preview();

    REQUIRE(run != nullptr);
    REQUIRE(run->parameter.size() == run->samples.size());
    CHECK(run->parameter.front().s == 0.0);
    CHECK(run->parameter.back().s == 1.0);
    for(std::size_t at = 1u; at < run->parameter.size(); ++at)
    {
        INFO("the sample the parameter turned back at: " << at);
        REQUIRE(run->parameter[at].s >= run->parameter[at - 1u].s);
    }
}

// The identity that makes the parameter the same quantity in both readings: a joint straight line
// composed with a chosen scaling travels the scaling times its whole displacement, so the parameter
// it realizes is that scaling.
TEST_CASE("the realized parameter of a composed motion is the very scaling it was composed under", "[manipulator][trajectory]")
{
    const std::pair<const char *, time_scaling_choice> composed[]{
            {"cubic", time_scaling_choice::cubic}, {"quintic", time_scaling_choice::quintic}, {"trapezoidal", time_scaling_choice::trapezoidal}};

    for(const auto &[named, chosen] : composed)
    {
        INFO("the scaling the motion was composed under: " << named);

        const previewed_arm arm = arm_at(configuration(0.0, 0.0));
        arm.control->set_time_scaling(chosen);
        arm.control->preview_trajectory(configuration(0.4, -0.2));

        const std::shared_ptr<const preview_run> run = arm.control->preview();

        REQUIRE(run != nullptr);
        REQUIRE(run->scaling.size() == 3u);

        const std::vector<trajectory::scaling_sample> &candidate = run->scaling[static_cast<std::size_t>(chosen)];

        REQUIRE(candidate.size() == run->parameter.size());
        for(std::size_t at = 0; at < candidate.size(); ++at)
        {
            INFO("the sample the two parted at: " << at);
            REQUIRE(is_approx_equal(run->parameter[at].s, candidate[at].s, reproduced_within));
            REQUIRE(is_approx_equal(run->parameter[at].ds, candidate[at].ds, reproduced_within));
            REQUIRE(is_approx_equal(run->parameter[at].dds, candidate[at].dds, reproduced_within));
        }
    }
}

// The difference between coming to rest at every row and flowing through them, read off one quantity:
// a spline through the rows keeps moving at each of them and only the two ends are at rest.
TEST_CASE("the realized parameter of a via-point run does not come to rest at an interior waypoint", "[manipulator][trajectory]")
{
    const previewed_arm arm                   = arm_at(configuration(0.0, 0.0));
    const std::vector<joint_vector> waypoints = {configuration(0.2, 0.3), configuration(-0.1, 0.5), configuration(0.5, 0.1), configuration(0.4, -0.2)};

    arm.control->preview_trajectory(std::span<const joint_vector>(waypoints));

    const std::shared_ptr<const preview_run> run = arm.control->preview();

    REQUIRE(run != nullptr);
    REQUIRE(run->parameter.size() == run->samples.size());
    CHECK(run->parameter.front().ds < at_rest_below);
    CHECK(run->parameter.back().ds < at_rest_below);

    for(std::size_t which = 1u; which + 1u < waypoints.size(); ++which)
    {
        INFO("the interior waypoint the run came to rest at: " << which);
        REQUIRE(run->parameter[stood_nearest(*run, waypoints[which])].ds > moving_above);
    }
}

TEST_CASE("the realized parameter of a queued run comes to rest at every leg boundary and resets there", "[manipulator][trajectory]")
{
    const previewed_arm arm = arm_at(configuration(0.0, 0.0));
    const std::vector<joint_vector> rows{configuration(0.2, 0.3), configuration(-0.1, 0.5), configuration(0.5, 0.1), configuration(0.4, -0.2)};

    arm.control->preview_in_turn(rows);

    const std::shared_ptr<const preview_run> run = arm.control->preview();

    REQUIRE(run != nullptr);
    REQUIRE(run->samples.size() == 3u * previewed_samples);
    REQUIRE(run->parameter.size() == run->samples.size());

    for(std::size_t leg = 0; leg < rows.size() - 1u; ++leg)
    {
        INFO("the leg the parameter did not run whole over: " << leg);
        REQUIRE(run->parameter[leg * previewed_samples].s == 0.0);
        REQUIRE(run->parameter[leg * previewed_samples].ds < at_rest_below);
        REQUIRE(run->parameter[(leg + 1u) * previewed_samples - 1u].s == 1.0);
        REQUIRE(run->parameter[(leg + 1u) * previewed_samples - 1u].ds < at_rest_below);
        REQUIRE(run->parameter[leg * previewed_samples + previewed_samples / 2u].ds > moving_above);
    }
}

TEST_CASE("a run of one sample carries no realized parameter rather than a quotient by nothing", "[manipulator][trajectory]")
{
    CHECK(realized_parameter({}).empty());

    const preview_sample alone{0.0, trajectory::trajectory_sample{configuration(0.2, 0.3), configuration(0.0, 0.0), configuration(0.0, 0.0)}, transform::Identity()};
    const std::vector<preview_sample> one{alone};
    const std::vector<preview_sample> two{alone, alone};

    CHECK(realized_parameter(one).empty());
    CHECK(realized_parameter(two).empty());
}

// The sampling of a task-space preview enters the solver once per sample, and none of those entries
// is a request the arm answers for: what a learner reads afterwards is what they read before.
TEST_CASE("a task-space preview leaves the published iterate sequences exactly as they were", "[manipulator][trajectory]")
{
    const previewed_arm arm = arm_at(configuration(0.0, 0.0));

    arm.control->preview_task_space_pose(reachable(0.3, 0.0));

    const std::vector<std::vector<iteration_state>> before(arm.control->solves().begin(), arm.control->solves().end());
    const std::vector<joint_vector> answered(arm.control->solutions().begin(), arm.control->solutions().end());

    REQUIRE(before.size() == 1u);

    arm.control->preview_trajectory(reachable(0.4, -0.2));

    REQUIRE(arm.control->preview() != nullptr);
    REQUIRE(arm.control->solves().size() == before.size());
    CHECK(arm.control->solves().front().size() == before.front().size());
    CHECK(arm.control->solutions().size() == answered.size());
}

// The run is opened where a run begins rather than where each of its motions is loaded, so one
// motion asked for on its own opens exactly one and a queued run of three targets leaves one path
// across all three rather than the leg that happened to be last.
TEST_CASE("the traversed run records where the tool went, is opened again by the next motion, and spans every leg of a queued run", "[manipulator][trajectory]")
{
    const previewed_arm arm = arm_at(configuration(0.0, 0.0));

    CHECK(arm.control->traversed() == nullptr);

    arm.control->task_space_ptp(reachable(0.4, -0.2));
    REQUIRE(played_out(*arm.control));

    const std::shared_ptr<const std::vector<transform>> run = arm.control->traversed();

    REQUIRE(run != nullptr);
    CHECK(run->size() > 1u);
    CHECK(is_approx_equal(run->back(), arm.robot->tool_pose().value(), 1.0e-9));
    CHECK(is_approx_equal(arm.robot->joint_positions(), configuration(0.4, -0.2), 1.0e-9));

    arm.control->task_space_ptp(reachable(0.1, 0.1));
    CHECK(arm.control->traversed() == nullptr);

    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const arm_pipe queued                   = pipe(loop);
    const std::weak_ptr<owned_arm> observer = queued.owned;
    const std::vector<joint_vector> targets{configuration(0.3, 0.1), configuration(-0.2, 0.4), configuration(0.5, -0.3)};

    command(observer, [](robot_controller &control, scene_robot &) { control.set_velocity_factor(1.0); });
    command(observer, [targets](robot_controller &control, scene_robot &) { control.run_in_turn(targets); });
    REQUIRE(loop.drain().has_value());
    REQUIRE(ticked_out(loop, *queued.control));

    const std::shared_ptr<const std::vector<transform>> whole = queued.control->traversed();

    REQUIRE(whole != nullptr);
    for(const joint_vector &target : targets)
    {
        INFO("the target the run should have passed through: " << target.transpose());
        CHECK(stood_at(*whole, queued.robot->tool_pose_at(target).value()));
    }
}

// The comparison the run exists for is the last motion's traversed path against the next motion's
// commanded one, so neither taking a preview nor standing the arm through one may empty it.
TEST_CASE("neither a preview nor a scrub clears the traversed run standing before them", "[manipulator][trajectory]")
{
    const previewed_arm arm = arm_at(configuration(0.0, 0.0));

    arm.control->task_space_ptp(reachable(0.4, -0.2));
    REQUIRE(played_out(*arm.control));

    const std::shared_ptr<const std::vector<transform>> run = arm.control->traversed();

    REQUIRE(run != nullptr);

    arm.control->preview_trajectory(configuration(0.1, 0.1));
    REQUIRE(arm.control->preview() != nullptr);
    CHECK(arm.control->traversed().get() == run.get());

    arm.control->preview_joint_configuration(configuration(0.2, 0.2));
    CHECK(arm.control->traversed().get() == run.get());
}

TEST_CASE("a publication carries both runs whole and puts the preview back when it is cleared", "[manipulator][trajectory]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const arm_pipe arm                              = pipe(loop);
    const std::weak_ptr<owned_arm> observer         = arm.owned;
    const std::shared_ptr<const arm_snapshot> quiet = arm.seen.read();

    CHECK(quiet->preview == nullptr);
    CHECK(quiet->traversed == nullptr);
    CHECK(quiet->time_scaling == time_scaling_choice::quintic);
    CHECK_FALSE(quiet->trapezoid.has_value());

    command(observer, [](robot_controller &control, scene_robot &) { control.preview_trajectory(configuration(0.4, -0.2)); });
    REQUIRE(loop.drain().has_value());

    const std::shared_ptr<const arm_snapshot> previewed = arm.seen.read();

    REQUIRE(previewed->preview != nullptr);
    CHECK(previewed->preview->samples.size() == previewed_samples);

    command(observer, [](robot_controller &control, scene_robot &) { control.clear_preview(); });
    REQUIRE(loop.drain().has_value());

    CHECK(arm.seen.read()->preview == nullptr);
}

// A publication copying either run rather than sharing it would cost the whole run on every playback
// step; pointer identity across two publications is what says it did not.
TEST_CASE("two publications with neither run changed between them share both handles", "[manipulator][trajectory]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const arm_pipe arm                      = pipe(loop);
    const std::weak_ptr<owned_arm> observer = arm.owned;

    command(observer, [](robot_controller &control, scene_robot &) { control.preview_trajectory(configuration(0.4, -0.2)); });
    command(observer, [](robot_controller &control, scene_robot &) { control.task_space_ptp(reachable(0.4, -0.2)); });
    command(observer, [](robot_controller &control, scene_robot &) { static_cast<void>(control.advance_playback(stepped)); });
    REQUIRE(loop.drain().has_value());

    const std::shared_ptr<const arm_snapshot> moving = arm.seen.read();

    REQUIRE(moving->preview != nullptr);
    REQUIRE(moving->traversed != nullptr);

    command(observer, [](robot_controller &control, scene_robot &) { control.set_velocity_factor(0.5); });
    REQUIRE(loop.drain().has_value());

    const std::shared_ptr<const arm_snapshot> settled = arm.seen.read();

    REQUIRE(settled != moving);
    CHECK(settled->velocity_factor == 0.5);
    CHECK(settled->preview.get() == moving->preview.get());
    CHECK(settled->traversed.get() == moving->traversed.get());
}

// A play that recomposed the same request would run for the span its own composition settled on,
// and from where a scrub left the arm that is not the span the preview reports.
TEST_CASE("a preview scrubbed away from its start plays back as the motion it was sampled from", "[manipulator][trajectory]")
{
    const previewed_arm arm = arm_at(configuration(0.0, 0.0));

    arm.control->preview_trajectory(configuration(0.4, -0.2));

    const std::shared_ptr<const preview_run> run = arm.control->preview();

    REQUIRE(run != nullptr);

    const std::size_t middle = run->samples.size() / 2u;
    arm.control->preview_joint_configuration(run->samples[middle].motion.position);
    REQUIRE_FALSE(is_approx_equal(arm.robot->joint_positions(), run->samples.front().motion.position, 1.0e-9));

    arm.control->play_preview();

    REQUIRE(arm.control->executing());
    CHECK(is_approx_equal(arm.robot->joint_positions(), run->samples.front().motion.position, 1.0e-12));

    const std::uint32_t steps = steps_of(*arm.control);

    CHECK_FALSE(arm.control->executing());
    CHECK(is_approx_equal(arm.robot->joint_positions(), run->samples.back().motion.position, 1.0e-9));
    CHECK(static_cast<double>(steps) * played_per_step(*arm.control) >= run->span);
    CHECK(static_cast<double>(steps - 1u) * played_per_step(*arm.control) < run->span);

    // What a second composition of the same request would have run for, from where the scrub left
    // the arm, which is what the count above would have measured had the play recomposed.
    const previewed_arm again = arm_at(run->samples[middle].motion.position);
    again.control->preview_trajectory(configuration(0.4, -0.2));

    REQUIRE(again.control->preview() != nullptr);
    CHECK(again.control->preview()->span < run->span);
}

TEST_CASE("playing where no previewed motion stands says so and leaves the composition able to preview again", "[manipulator][trajectory]")
{
    const previewed_arm arm = arm_at(configuration(0.0, 0.0));

    const std::string reported = praxis::tests::reported_by([&] { arm.control->play_preview(); });

    CHECK_FALSE(arm.control->executing());
    CHECK_THAT(reported, Catch::Matchers::ContainsSubstring("play"));

    arm.control->preview_trajectory(configuration(0.4, -0.2));

    CHECK(arm.control->preview() != nullptr);
}

TEST_CASE("playing while a motion is already executing does nothing, and the same preview plays once it has ended", "[manipulator][trajectory]")
{
    const previewed_arm arm = arm_at(configuration(0.0, 0.0));

    arm.control->preview_trajectory(configuration(0.4, -0.2));
    REQUIRE(arm.control->preview() != nullptr);

    arm.control->task_space_ptp(reachable(0.2, 0.1));
    REQUIRE(arm.control->executing());
    for(std::uint32_t taken = 0; taken < 50u; ++taken)
        static_cast<void>(arm.control->advance_playback(stepped));

    const joint_vector playing = arm.robot->joint_positions();

    arm.control->play_preview();

    CHECK(is_approx_equal(arm.robot->joint_positions(), playing, 1.0e-12));
    REQUIRE(played_out(*arm.control));

    arm.control->play_preview();

    CHECK(arm.control->executing());
}

// The keep ends at the play, because the executor takes the generator; the sampled run stays
// published, which is what leaves the commanded path drawn while the tool traverses it.
TEST_CASE("the preview a play consumed stays published, and clearing it afterwards leaves the arm where the motion left it", "[manipulator][trajectory]")
{
    const previewed_arm arm = arm_at(configuration(0.0, 0.0));

    arm.control->preview_trajectory(configuration(0.4, -0.2));

    const std::shared_ptr<const preview_run> run = arm.control->preview();

    REQUIRE(run != nullptr);

    arm.control->play_preview();
    REQUIRE(played_out(*arm.control));

    CHECK(arm.control->preview().get() == run.get());

    arm.control->play_preview();

    CHECK_FALSE(arm.control->executing());

    const joint_vector ended = arm.robot->joint_positions();
    arm.control->clear_preview();

    CHECK(arm.control->preview() == nullptr);
    CHECK(is_approx_equal(arm.robot->joint_positions(), ended, 1.0e-12));
}

// Each target is a motion of its own, so the arm reaches every one of them exactly: the next is
// loaded on the tick the one before it concluded, and loading takes no sample.
TEST_CASE("a run of targets plays each in turn and stands at every one of them", "[manipulator][trajectory]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const arm_pipe arm                      = pipe(loop);
    const std::weak_ptr<owned_arm> observer = arm.owned;
    const std::vector<joint_vector> targets{configuration(0.3, 0.1), configuration(-0.2, 0.4), configuration(0.5, -0.3)};

    command(observer, [](robot_controller &control, scene_robot &) { control.set_velocity_factor(1.0); });
    command(observer, [targets](robot_controller &control, scene_robot &) { control.run_in_turn(targets); });
    REQUIRE(loop.drain().has_value());

    REQUIRE(arm.control->executing());
    REQUIRE(arm.control->queued().size() == 2u);

    std::vector<joint_vector> stood;
    std::size_t left = arm.control->queued().size();
    for(std::uint32_t taken = 0; taken < most_steps && arm.control->executing(); ++taken)
    {
        dictated += std::chrono::duration_cast<time_point::duration>(stepped.value);
        REQUIRE(loop.drain().has_value());
        if(arm.control->queued().size() != left)
        {
            stood.push_back(arm.robot->joint_positions());
            left = arm.control->queued().size();
        }
    }

    REQUIRE_FALSE(arm.control->executing());
    REQUIRE(stood.size() == 2u);
    CHECK(is_approx_equal(stood[0], targets[0], 1.0e-12));
    CHECK(is_approx_equal(stood[1], targets[1], 1.0e-12));
    CHECK(is_approx_equal(arm.robot->joint_positions(), targets[2], 1.0e-12));
}

TEST_CASE("clearing the preview abandons a queued run, and nothing follows the target in flight", "[manipulator][trajectory]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const arm_pipe arm                      = pipe(loop);
    const std::weak_ptr<owned_arm> observer = arm.owned;
    const std::vector<joint_vector> targets{configuration(0.3, 0.1), configuration(-0.2, 0.4), configuration(0.5, -0.3)};

    command(observer, [](robot_controller &control, scene_robot &) { control.set_velocity_factor(1.0); });
    command(observer, [targets](robot_controller &control, scene_robot &) { control.run_in_turn(targets); });
    REQUIRE(loop.drain().has_value());
    REQUIRE(arm.control->queued().size() == 2u);

    command(observer, [](robot_controller &control, scene_robot &) { control.clear_preview(); });
    REQUIRE(loop.drain().has_value());

    CHECK(arm.control->queued().empty());
    REQUIRE(ticked_out(loop, *arm.control));
    CHECK(arm.control->queued().empty());
    CHECK(is_approx_equal(arm.robot->joint_positions(), targets.front(), 1.0e-12));
}

// The shipped composition, with a way to say the composition cannot continue bound to it. A target a
// learner typed is user-driven input, so a factory that refuses one reports and returns rather than
// asking the composition to unload.
TEST_CASE("a queued run whose next target the composition refuses stops there, names it, and leaves the composition standing", "[manipulator][trajectory]")
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const strand work   = *loop.make_strand();
    previewed_arm built = arm_at(configuration(0.0, 0.0));
    auto published      = std::make_shared<arm_publisher>();

    bool unloaded = false;
    auto control =
            std::make_shared<robot_controller>(*built.robot, motion_ops{.task_space_pose = &solving_task_space_pose}, trajectory::baseline().path, task_trajectory_ops{},
                                               trajectory::baseline().time_scaling, trajectory::baseline().trajectory, rigid_motion::screw_ops{}, [&unloaded] { unloaded = true; });
    const auto owned                        = std::make_shared<owned_arm>(work, work, built.robot, control, published);
    const std::weak_ptr<owned_arm> observer = owned;

    joint_vector too_wide(3);
    too_wide << 0.1, 0.2, 0.3;
    const std::vector<joint_vector> targets{configuration(0.3, 0.1), too_wide, configuration(0.5, -0.3)};

    command(observer, [](robot_controller &driven, scene_robot &) { driven.set_velocity_factor(1.0); });
    command(observer, [targets](robot_controller &driven, scene_robot &) { driven.run_in_turn(targets); });
    REQUIRE(loop.drain().has_value());
    REQUIRE(control->executing());

    const std::string reported = praxis::tests::reported_by([&] { REQUIRE(ticked_out(loop, *control)); });

    CHECK_THAT(reported, Catch::Matchers::ContainsSubstring("2"));
    CHECK(control->queued().empty());
    CHECK_FALSE(control->executing());
    CHECK_FALSE(unloaded);
    CHECK(is_approx_equal(built.robot->joint_positions(), targets.front(), 1.0e-12));

    // The composition is still usable: the target after the refused one is played when it is asked
    // for on its own.
    command(observer, [](robot_controller &driven, scene_robot &) { driven.run_in_turn(std::vector<joint_vector>{configuration(0.5, -0.3)}); });
    REQUIRE(loop.drain().has_value());

    CHECK(control->executing());
}
