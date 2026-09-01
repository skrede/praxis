#include "fixtures.h"

#include "captured_log.h"

#include "praxis/manipulator/robot_controller.h"

#include "praxis/evaluation/tolerance.h"

#include "praxis/scene/log_buffer.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/trajectory/capabilities.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <spdlog/spdlog.h>

#include <cmath>
#include <array>
#include <chrono>
#include <memory>
#include <vector>
#include <cstdint>
#include <fstream>
#include <utility>
#include <optional>
#include <stdexcept>
#include <filesystem>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

constexpr std::uint32_t repeated_drives = 5;

static_assert(repeated_drives < scene::default_log_capacity, "no drive may be dropped before the drain");

scene_robot adapter(const robot_ops &injected)
{
    return two_joint_arm(injected);
}

struct recording_outcome
{
    expected<std::filesystem::path, refusal> answer = std::filesystem::path();
    std::string diagnosis;
};

recording_outcome recording_to(robot_controller &controller, const std::filesystem::path &folder)
{
    recording_parameters parameters;
    parameters.active    = true;
    parameters.directory = folder;

    expected<std::filesystem::path, refusal> answer = std::filesystem::path();
    std::string diagnosis                           = reported_by([&] { answer = controller.set_recording(parameters); });

    return recording_outcome{std::move(answer), std::move(diagnosis)};
}

transform planar_pose(double x, double y)
{
    transform pose = transform::Identity();
    pose(0, 3)     = x;
    pose(1, 3)     = y;

    return pose;
}

void drive_refused_pose(robot_controller &controller, std::uint32_t times)
{
    for(std::uint32_t drive = 0; drive < times; ++drive)
        controller.preview_task_space_pose(planar_pose(0.9, -0.7));
}

std::shared_ptr<scene::log_buffer> ring_on_the_current_logger()
{
    auto messages = std::make_shared<scene::log_buffer>(scene::default_log_capacity);
    scene::install_log_sink(messages);

    return messages;
}

expected<joint_vector, refusal> screw_angle_as_configuration(const rigid_motion::screw_ops &, const kinematics &, const transform &, const Eigen::Vector3d &, const Eigen::Vector3d &,
                                                             double theta, double, const joint_vector &)
{
    return configuration(theta, -theta);
}

motion_ops previewing_motion()
{
    return motion_ops{.task_space_pose = &position_as_configuration, .task_space_screw = &screw_angle_as_configuration};
}

// A span long enough for the playback loop to reach its first sample, which is where the refusal
// this pair of cases is about arrives.
class refusing_trajectory : public trajectory::trajectory_generator
{
public:
    expected<trajectory::trajectory_sample, refusal> sample(double) const override
    {
        return praxis::unexpected(refusal::not_implemented);
    }

    double duration() const override
    {
        return 1.0;
    }
};

expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> refusing_waypoints(std::span<const trajectory::configuration>, const trajectory::configuration &,
                                                                                        const trajectory::configuration_limits &)
{
    return std::make_unique<refusing_trajectory>();
}

expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> refused_waypoints(std::span<const trajectory::configuration>, const trajectory::configuration &,
                                                                                       const trajectory::configuration_limits &)
{
    return praxis::unexpected(refusal::unsupported_input);
}

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

constexpr step_period stepped{seconds{0.01}};
constexpr std::uint32_t most_steps = 100000;

// The controller no longer owns the stepping, so a case drives a motion the way the aggregate does:
// a stepped task on a strand of its own, and a dictated reading raised one period at a time. The
// bound is a failure report rather than a synchronization device.
bool played_out(robot_controller &controller)
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const strand work        = *loop.make_strand();
    const task_handle motion = work.every(stepped, overrun::catch_up, [&controller](step_delta step) { static_cast<void>(controller.advance_playback(step)); });

    for(std::uint32_t taken = 0; taken < most_steps && controller.executing(); ++taken)
    {
        dictated += std::chrono::duration_cast<time_point::duration>(stepped.value);
        if(!loop.drain().has_value())
            return false;
    }

    return !controller.executing();
}

// The duration a composition settled on is not published anywhere, so the reference scalings are
// bound behind slots that keep the duration they were sampled at on the way through.
double commanded_span = 0.0;

expected<trajectory::scaling_sample, refusal> recorded_cubic(double t, double duration)
{
    commanded_span = duration;

    return trajectory::baseline().time_scaling.cubic(t, duration);
}

expected<trajectory::scaling_sample, refusal> recorded_quintic(double t, double duration)
{
    commanded_span = duration;

    return trajectory::baseline().time_scaling.quintic(t, duration);
}

expected<trajectory::scaling_sample, refusal> recorded_trapezoidal(double t, double duration, double max_velocity, double max_acceleration)
{
    commanded_span = duration;

    return trajectory::baseline().time_scaling.trapezoidal(t, duration, max_velocity, max_acceleration);
}

trajectory::time_scaling_ops recording_time_scaling()
{
    return trajectory::time_scaling_ops{.cubic = &recorded_cubic, .quintic = &recorded_quintic, .trapezoidal = &recorded_trapezoidal};
}

double span_under(robot_controller &controller, scene_robot &robot, time_scaling_choice chosen)
{
    robot.set_joint_positions(configuration(0.0, 0.0));
    controller.set_time_scaling(chosen);
    commanded_span = 0.0;

    controller.task_space_ptp(planar_pose(0.4, -0.3));
    REQUIRE(played_out(controller));

    return commanded_span;
}

}

TEST_CASE("the_controller_reports_what_the_adapter_reports")
{
    scene_robot robot = adapter(robot_ops{});
    robot.set_joint_positions(configuration(0.3, -0.1));
    const robot_controller controller(robot, composing_motion(), composing_path(), task_trajectory_ops{}, composing_time_scaling(), trajectory::trajectory_ops{},
                                      rigid_motion::screw_ops{});

    CHECK(controller.joint_count() == robot.joint_count());
    CHECK(is_approx_equal(controller.joint_positions(), robot.joint_positions()));
    CHECK(is_approx_equal(controller.flange_pose().value(), robot.flange_pose().value()));
    CHECK(is_approx_equal(controller.tool_pose().value(), robot.tool_pose().value()));
    CHECK((controller.tool_orientation().value() - robot.tool_orientation().value()).isZero(default_tolerance));
    CHECK(is_approx_equal(controller.limits().velocity[0], robot.limits().velocity[0]));
}

TEST_CASE("the_recording_names_no_directory_until_one_is_given")
{
    const scratch_tree scratch("robot_controller");
    scene_robot robot = adapter(robot_ops{});
    robot_controller controller(robot, motion_ops{}, trajectory::path_ops{}, task_trajectory_ops{}, trajectory::time_scaling_ops{}, trajectory::trajectory_ops{},
                                rigid_motion::screw_ops{});

    CHECK_FALSE(controller.recording().active);
    CHECK(controller.recording().directory.empty());

    const std::filesystem::path folder = scratch.root() / "recordings";

    REQUIRE(recording_to(controller, folder).answer.has_value());
    CHECK(controller.recording().active);
    CHECK(controller.recording().directory == std::filesystem::weakly_canonical(folder));
}

// Naming no folder is not naming one that could not be prepared. With no root to resolve against, an
// unset folder resolves to nothing and there is nothing to make.
TEST_CASE("recording settings that name no folder are accepted with nothing prepared and nothing said")
{
    scene_robot robot = adapter(robot_ops{});
    robot_controller controller(robot, motion_ops{}, trajectory::path_ops{}, task_trajectory_ops{}, trajectory::time_scaling_ops{}, trajectory::trajectory_ops{},
                                rigid_motion::screw_ops{});

    const recording_outcome outcome = recording_to(controller, std::filesystem::path());

    CHECK(outcome.answer.has_value());
    CHECK(outcome.diagnosis.empty());
    CHECK(controller.recording().active);
    CHECK(controller.recording().directory.empty());
}

TEST_CASE("a recording folder that does not exist yet is created before the settings are accepted")
{
    const scratch_tree scratch("robot_controller");
    scene_robot robot                  = adapter(robot_ops{});
    const std::filesystem::path folder = scratch.root() / "runs" / "today";
    robot_controller controller(robot, motion_ops{}, trajectory::path_ops{}, task_trajectory_ops{}, trajectory::time_scaling_ops{}, trajectory::trajectory_ops{},
                                rigid_motion::screw_ops{});

    CHECK(recording_to(controller, folder).answer.has_value());
    CHECK(std::filesystem::is_directory(folder));
    CHECK(controller.recording().directory == std::filesystem::weakly_canonical(folder));
}

TEST_CASE("a recording folder that already exists is accepted")
{
    const scratch_tree scratch("robot_controller");
    scene_robot robot                  = adapter(robot_ops{});
    const std::filesystem::path folder = scratch.root() / "existing";
    std::filesystem::create_directories(folder);
    robot_controller controller(robot, motion_ops{}, trajectory::path_ops{}, task_trajectory_ops{}, trajectory::time_scaling_ops{}, trajectory::trajectory_ops{},
                                rigid_motion::screw_ops{});

    CHECK(recording_to(controller, folder).answer.has_value());
    CHECK(controller.recording().directory == std::filesystem::weakly_canonical(folder));
}

// The refusal and the settings are one step: the folder that could not be prepared must not displace
// the one the recording is already writing into.
TEST_CASE("a recording folder that cannot be prepared is refused and leaves the settings it would have replaced")
{
    const scratch_tree scratch("robot_controller");
    scene_robot robot                    = adapter(robot_ops{});
    const std::filesystem::path settled  = scratch.root() / "settled";
    const std::filesystem::path occupied = scratch.root() / "occupied";
    std::ofstream(occupied) << "a regular file, not a folder";
    const std::filesystem::path beneath = occupied / "beneath";
    robot_controller controller(robot, motion_ops{}, trajectory::path_ops{}, task_trajectory_ops{}, trajectory::time_scaling_ops{}, trajectory::trajectory_ops{},
                                rigid_motion::screw_ops{});

    REQUIRE(recording_to(controller, settled).answer.has_value());

    const recording_outcome outcome = recording_to(controller, beneath);

    REQUIRE_FALSE(outcome.answer.has_value());
    CHECK(outcome.answer.error() == refusal::unsupported_input);
    CHECK_THAT(outcome.diagnosis, Catch::Matchers::ContainsSubstring(beneath.string()));
    CHECK_FALSE(std::filesystem::exists(beneath));
    CHECK(controller.recording().directory == std::filesystem::weakly_canonical(settled));
}

TEST_CASE("the_velocity_factor_lives_on_the_controller")
{
    scene_robot robot = adapter(robot_ops{});
    robot_controller controller(robot, motion_ops{}, trajectory::path_ops{}, task_trajectory_ops{}, trajectory::time_scaling_ops{}, trajectory::trajectory_ops{},
                                rigid_motion::screw_ops{});

    CHECK(is_approx_equal(controller.velocity_factor(), 0.3));
    controller.set_velocity_factor(0.75);
    CHECK(is_approx_equal(controller.velocity_factor(), 0.75));
}

TEST_CASE("a_previewed_configuration_moves_the_arm_without_running_a_motion")
{
    scene_robot robot = adapter(robot_ops{});
    robot_controller controller(robot, motion_ops{}, trajectory::path_ops{}, task_trajectory_ops{}, trajectory::time_scaling_ops{}, trajectory::trajectory_ops{},
                                rigid_motion::screw_ops{});

    controller.preview_joint_configuration(configuration(0.2, -0.4));

    CHECK_FALSE(controller.executing());
    CHECK(is_approx_equal(robot.joint_positions(), configuration(0.2, -0.4), round_trip));
}

TEST_CASE("the_second_of_two_previewed_screw_angles_is_the_one_the_arm_is_left_at")
{
    scene_robot robot = adapter(robot_ops{});
    robot_controller controller(robot, previewing_motion(), composing_path(), task_trajectory_ops{}, composing_time_scaling(), trajectory::trajectory_ops{}, rigid_motion::screw_ops{});

    controller.preview_task_space_screw(transform::Identity(), Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(), 0.3, 0.0);
    controller.preview_task_space_screw(transform::Identity(), Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(), 0.9, 0.0);

    CHECK_FALSE(controller.executing());
    CHECK(is_approx_equal(controller.joint_positions(), configuration(0.9, -0.9), round_trip));
}

TEST_CASE("a_point_to_point_command_plays_back_and_leaves_the_arm_at_the_target")
{
    scene_robot robot = adapter(robot_ops{});
    robot_controller controller(robot, composing_motion(), composing_path(), task_trajectory_ops{}, composing_time_scaling(), trajectory::trajectory_ops{}, rigid_motion::screw_ops{});
    controller.set_velocity_factor(8.0);

    controller.task_space_ptp(planar_pose(0.4, -0.3));

    REQUIRE(played_out(controller));
    CHECK(is_approx_equal(robot.joint_positions(), configuration(0.4, -0.3), round_trip));
}

// A refused sample is the one point where a consumer could quietly invent a configuration, so the
// arm's own reading is what the case asserts against rather than the executor's state.
TEST_CASE("a_refused_sample_stops_the_playback_and_leaves_the_arm_where_it_is")
{
    scene_robot robot = adapter(robot_ops{});
    robot_controller controller(robot, motion_ops{}, trajectory::path_ops{}, task_trajectory_ops{}, trajectory::time_scaling_ops{},
                                trajectory::trajectory_ops{.joint_space_waypoints = &refusing_waypoints}, rigid_motion::screw_ops{});
    robot.set_joint_positions(configuration(0.15, 0.25));

    controller.joint_space_trajectory(std::array<joint_vector, 1>{configuration(0.9, -0.9)});

    REQUIRE(played_out(controller));
    CHECK(is_approx_equal(robot.joint_positions(), configuration(0.15, 0.25), round_trip));
}

TEST_CASE("a_waypoint_factory_that_refuses_commands_no_motion_at_all")
{
    scene_robot robot = adapter(robot_ops{});
    robot_controller controller(robot, motion_ops{}, trajectory::path_ops{}, task_trajectory_ops{}, trajectory::time_scaling_ops{},
                                trajectory::trajectory_ops{.joint_space_waypoints = &refused_waypoints}, rigid_motion::screw_ops{});
    robot.set_joint_positions(configuration(0.15, 0.25));

    controller.joint_space_trajectory(std::array<joint_vector, 1>{configuration(0.9, -0.9)});

    CHECK_FALSE(controller.executing());
    CHECK(is_approx_equal(robot.joint_positions(), configuration(0.15, 0.25), round_trip));
}

// The comparison is exact rather than approximate: a consumer that substituted the seed, the
// identity or the last good answer would land within any tolerance loose enough to absorb the
// renderer's own float round trip.
TEST_CASE("a_preview_a_slot_refuses_leaves_the_arm_at_exactly_the_configuration_it_had")
{
    scene_robot robot = adapter(robot_ops{});
    robot_controller controller(robot, motion_ops{}, trajectory::path_ops{}, task_trajectory_ops{}, trajectory::time_scaling_ops{}, trajectory::trajectory_ops{},
                                rigid_motion::screw_ops{});
    robot.set_joint_positions(configuration(0.15, 0.25));
    const joint_vector before = robot.joint_positions();

    controller.preview_task_space_pose(planar_pose(0.9, -0.7));
    controller.preview_task_space_screw(transform::Identity(), Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(), 0.3, 0.0);
    controller.preview_tool_frame_jog(transform::Identity(), Eigen::Vector3d::UnitX(), rotation::Identity());

    CHECK_FALSE(controller.executing());
    CHECK((robot.joint_positions().array() == before.array()).all());
}

// A zero screw axis exponentiates to the identity, so a command that substituted one would drive the
// arm to the pose it already holds and leave the configuration untouched too. What separates the two
// is where the refusal is received: the arm standing still is necessary and the report is what says
// the command stopped at the construction rather than somewhere further down.
TEST_CASE("a_screw_command_whose_axis_construction_refuses_stops_there_and_moves_the_arm_by_nothing")
{
    scene_robot robot = adapter(robot_ops{});
    robot_controller controller(robot, composing_motion(), composing_path(), task_trajectory_ops{}, composing_time_scaling(), trajectory::trajectory_ops{}, rigid_motion::screw_ops{});
    robot.set_joint_positions(configuration(0.15, 0.25));
    const joint_vector before = robot.joint_positions();

    const std::string reported = reported_by([&] { controller.task_space_screw(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(), 0.3, 0.0); });

    CHECK_FALSE(controller.executing());
    CHECK((robot.joint_positions().array() == before.array()).all());
    CHECK_THAT(reported, Catch::Matchers::ContainsSubstring("screw_axis_from_point_direction_pitch"));
}

// The choice reaches a point-to-point command, which is a jog rather than a generated trajectory, so
// what it governs is every motion the arm makes and not only the ones a preset composes.
TEST_CASE("the_three_time_scalings_give_three_durations_over_the_same_pair_of_configurations")
{
    scene_robot robot = adapter(robot_ops{});
    robot_controller controller(robot, composing_motion(), composing_path(), task_trajectory_ops{}, recording_time_scaling(), trajectory::trajectory_ops{}, rigid_motion::screw_ops{});
    controller.set_velocity_factor(8.0);

    CHECK(controller.time_scaling() == time_scaling_choice::quintic);

    const double cubic       = span_under(controller, robot, time_scaling_choice::cubic);
    const double quintic     = span_under(controller, robot, time_scaling_choice::quintic);
    const double trapezoidal = span_under(controller, robot, time_scaling_choice::trapezoidal);

    INFO("cubic " << cubic << " quintic " << quintic << " trapezoidal " << trapezoidal);
    CHECK(cubic > 0.0);
    CHECK(std::abs(cubic - quintic) > 1.0e-6);
    CHECK(std::abs(cubic - trapezoidal) > 1.0e-6);
    CHECK(std::abs(quintic - trapezoidal) > 1.0e-6);
    CHECK(controller.time_scaling() == time_scaling_choice::trapezoidal);
}

// An override the controller carries is what the composition holds the profile to, so the same pair
// under tighter bounds takes longer than the pair the arm's own limits gave.
TEST_CASE("an_overridden_trapezoid_bound_reaches_the_motion_the_controller_commands")
{
    scene_robot robot = adapter(robot_ops{});
    robot_controller controller(robot, composing_motion(), composing_path(), task_trajectory_ops{}, recording_time_scaling(), trajectory::trajectory_ops{}, rigid_motion::screw_ops{});
    controller.set_velocity_factor(8.0);

    const double derived = span_under(controller, robot, time_scaling_choice::trapezoidal);
    REQUIRE(controller.trapezoid_bounds() == std::nullopt);

    controller.set_trapezoid_bounds(path_parameter_bounds{0.5, 1.0});
    const double held = span_under(controller, robot, time_scaling_choice::trapezoidal);

    INFO("derived " << derived << " held " << held);
    CHECK(held > derived);
    CHECK(controller.trapezoid_bounds().has_value());
}

TEST_CASE("a_via_point_factory_that_refuses_commands_no_task_space_motion_at_all")
{
    scene_robot robot = adapter(robot_ops{});
    robot_controller controller(robot, motion_ops{}, trajectory::path_ops{}, task_trajectory_ops{}, trajectory::time_scaling_ops{}, trajectory::trajectory_ops{},
                                rigid_motion::screw_ops{});
    robot.set_joint_positions(configuration(0.15, 0.25));
    const joint_vector before = robot.joint_positions();

    controller.task_space_trajectory(std::array<transform, 1>{planar_pose(0.9, -0.7)});

    CHECK_FALSE(controller.executing());
    CHECK((robot.joint_positions().array() == before.array()).all());
}

TEST_CASE("a_point_to_point_command_a_resolution_refuses_starts_no_motion_and_moves_nothing")
{
    scene_robot robot = adapter(robot_ops{});
    robot_controller controller(robot, motion_ops{}, composing_path(), task_trajectory_ops{}, composing_time_scaling(), trajectory::trajectory_ops{}, rigid_motion::screw_ops{});
    robot.set_joint_positions(configuration(0.15, 0.25));
    const joint_vector before = robot.joint_positions();

    controller.task_space_ptp(planar_pose(0.9, -0.7));

    CHECK_FALSE(controller.executing());
    CHECK((robot.joint_positions().array() == before.array()).all());
}

TEST_CASE("a_command_on_unbound_capabilities_is_accepted_and_the_arm_does_not_move")
{
    scene_robot robot = adapter(robot_ops{});
    robot_controller controller(robot, motion_ops{}, trajectory::path_ops{}, task_trajectory_ops{}, trajectory::time_scaling_ops{}, trajectory::trajectory_ops{},
                                rigid_motion::screw_ops{});
    robot.set_joint_positions(configuration(0.15, 0.25));

    controller.task_space_ptp(planar_pose(0.9, 0.9));
    controller.task_space_lin(planar_pose(0.9, 0.9));

    REQUIRE(played_out(controller));
    CHECK(is_approx_equal(robot.joint_positions(), configuration(0.15, 0.25), round_trip));
}

// The installer appends its sink to whatever logger is default at the call and never removes it, so
// the ring goes onto the capture's logger and out with it, and the capture's own sink stays in place
// beneath it — which is what leaves the message on both surfaces.
TEST_CASE("a command refused over and over is one ring entry carrying its count, and the terminal carries the same text")
{
    scene_robot robot = adapter(robot_ops{});
    robot_controller controller(robot, motion_ops{}, trajectory::path_ops{}, task_trajectory_ops{}, trajectory::time_scaling_ops{}, trajectory::trajectory_ops{},
                                rigid_motion::screw_ops{});

    const captured_log terminal;
    const std::shared_ptr<scene::log_buffer> messages = ring_on_the_current_logger();

    drive_refused_pose(controller, repeated_drives);

    const std::vector<scene::log_entry> drained = messages->drain();

    REQUIRE(drained.size() == 1);
    CHECK(drained.front().repeats == repeated_drives);
    CHECK(drained.front().level == scene::severity::warning);
    CHECK_THAT(drained.front().text, Catch::Matchers::ContainsSubstring("motion.task_space_pose"));
    CHECK_THAT(terminal.text(), Catch::Matchers::ContainsSubstring(drained.front().text));
}

TEST_CASE("a differently named refusal between the repeats starts a new entry rather than raising one count")
{
    scene_robot robot = adapter(robot_ops{});
    robot_controller controller(robot, motion_ops{}, trajectory::path_ops{}, task_trajectory_ops{}, trajectory::time_scaling_ops{}, trajectory::trajectory_ops{},
                                rigid_motion::screw_ops{});

    const captured_log terminal;
    const std::shared_ptr<scene::log_buffer> messages = ring_on_the_current_logger();

    drive_refused_pose(controller, repeated_drives);
    controller.preview_tool_frame_jog(transform::Identity(), Eigen::Vector3d::UnitX(), rotation::Identity());
    drive_refused_pose(controller, repeated_drives);

    const std::vector<scene::log_entry> drained = messages->drain();

    REQUIRE(drained.size() == 3);
    CHECK(drained[0].repeats == repeated_drives);
    CHECK(drained[1].repeats == 1);
    CHECK(drained[2].repeats == repeated_drives);
    CHECK_THAT(drained[0].text, Catch::Matchers::ContainsSubstring("motion.task_space_pose"));
    CHECK_THAT(drained[1].text, Catch::Matchers::ContainsSubstring("motion.tool_frame_displace"));
    CHECK_THAT(terminal.text(), Catch::Matchers::ContainsSubstring(drained[1].text));
}

TEST_CASE("a_captured_command_that_leaves_by_a_throw_still_puts_the_previous_logger_back")
{
    const auto previous = spdlog::default_logger();

    CHECK_THROWS_AS(reported_by([] { throw std::runtime_error("out of a captured command"); }), std::runtime_error);

    CHECK(spdlog::default_logger() == previous);
}
