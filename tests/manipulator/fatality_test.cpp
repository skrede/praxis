#include "fixtures.h"
#include "captured_log.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/robot_controller.h"

#include "praxis/trajectory/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <memory>
#include <string>
#include <cstddef>
#include <vector>
#include <utility>
#include <functional>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

expected<joint_vector, refusal> unsupported(const kinematics &, const transform &, const joint_vector &)
{
    return praxis::unexpected(refusal::unsupported_input);
}

expected<joint_vector, refusal> ill_formed(const kinematics &, const transform &, const joint_vector &)
{
    return praxis::unexpected(refusal::degenerate);
}

expected<joint_vector, refusal> unreachable(const kinematics &, const transform &, const joint_vector &)
{
    return praxis::unexpected(refusal::no_solution);
}

robot_controller controlling(scene_robot &driven, const motion_ops &injected, std::function<void(std::string)> ask_unload)
{
    return robot_controller(driven, injected, trajectory::path_ops{}, task_trajectory_ops{}, trajectory::time_scaling_ops{}, trajectory::trajectory_ops{}, rigid_motion::screw_ops{},
                            rigid_motion::frame_ops{}, std::move(ask_unload));
}

// The factories a preset ships rather than a stub written to refuse, so what a case reads is the
// classification a shipped composition reaches. Each serves one or two waypoints and answers any
// other count with a refusal.
robot_controller shipping_waypoints(scene_robot &driven, std::function<void(std::string)> ask_unload)
{
    return robot_controller(driven, motion_ops{}, trajectory::path_ops{}, manipulator::baseline().trajectory, trajectory::time_scaling_ops{}, trajectory::baseline().trajectory,
                            rigid_motion::screw_ops{}, rigid_motion::frame_ops{}, std::move(ask_unload));
}

struct outcome
{
    int asked_to_unload;
    bool configuration_untouched;
};

// The controller is handed the route a composition's site offers, so what the case counts is how
// many times the decision reached for it rather than what a flag it shares was left holding.
outcome previewing(const motion_ops &injected)
{
    int asked          = 0;
    scene_robot driven = two_joint_arm(robot_ops{});
    driven.set_joint_positions(configuration(0.15, -0.25));
    const joint_vector before = driven.joint_positions();

    robot_controller controller = controlling(driven, injected, [&asked](std::string) { ++asked; });
    controller.preview_task_space_pose(transform::Identity());

    return outcome{asked, (driven.joint_positions().array() == before.array()).all()};
}

// The same arm reached through the joint-space preview instead, whose only refusal is the one the
// length disagreement raises: no binding is injected because none is consulted on that path.
outcome previewing_configuration(const joint_vector &positions)
{
    int asked          = 0;
    scene_robot driven = two_joint_arm(robot_ops{});
    driven.set_joint_positions(configuration(0.15, -0.25));
    const joint_vector before = driven.joint_positions();

    robot_controller controller = controlling(driven, motion_ops{}, [&asked](std::string) { ++asked; });
    controller.preview_joint_configuration(positions);

    const joint_vector after = driven.joint_positions();

    return outcome{asked, after.size() == before.size() && (after.array() == before.array()).all()};
}

outcome commanding_configurations(std::span<const joint_vector> waypoints)
{
    int asked          = 0;
    scene_robot driven = two_joint_arm(robot_ops{});
    driven.set_joint_positions(configuration(0.15, -0.25));
    const joint_vector before = driven.joint_positions();

    robot_controller controller = shipping_waypoints(driven, [&asked](std::string) { ++asked; });
    controller.joint_space_trajectory(waypoints);

    return outcome{asked, (driven.joint_positions().array() == before.array()).all()};
}

outcome commanding_poses(std::span<const transform> waypoints)
{
    int asked          = 0;
    scene_robot driven = two_joint_arm(robot_ops{});
    driven.set_joint_positions(configuration(0.15, -0.25));
    const joint_vector before = driven.joint_positions();

    robot_controller controller = shipping_waypoints(driven, [&asked](std::string) { ++asked; });
    controller.task_space_trajectory(waypoints);

    return outcome{asked, (driven.joint_positions().array() == before.array()).all()};
}

joint_vector three_joints()
{
    joint_vector held(3);
    held << 0.15, -0.25, 0.35;

    return held;
}

std::size_t occurrences(const std::string &reported, const std::string &of)
{
    std::size_t counted = 0;
    for(std::size_t at = reported.find(of); at != std::string::npos; at = reported.find(of, at + 1u))
        ++counted;

    return counted;
}

// A preview of a pose the fatal kind refuses, made over and over, as one dragged through a range of
// refused poses is.
std::string reported_over(int requests, const motion_ops &injected)
{
    int asked          = 0;
    scene_robot driven = two_joint_arm(robot_ops{});

    robot_controller controller = controlling(driven, injected, [&asked](std::string) { ++asked; });

    return praxis::tests::reported_by(
            [&]
            {
                for(int request = 0; request < requests; ++request)
                    controller.preview_task_space_pose(transform::Identity());
            });
}

}

// A preview is formed from a pose an operator is moving and is re-issued at every move of it, so an
// ill-formed one is what was asked for and says nothing about what answered. Each fatal kind is
// asserted on its own, so reclassifying one of them cannot pass on the strength of the other.
TEST_CASE("a_request_formed_from_a_value_an_operator_is_editing_unloads_nothing_whichever_fatal_kind_it_carries")
{
    const outcome unserved         = previewing(motion_ops{.task_space_pose = &unsupported});
    const outcome ill_formed_input = previewing(motion_ops{.task_space_pose = &ill_formed});

    CHECK(unserved.asked_to_unload == 0);
    CHECK(unserved.configuration_untouched);
    CHECK(ill_formed_input.asked_to_unload == 0);
    CHECK(ill_formed_input.configuration_untouched);
}

// The log ring carries a repeat count, so a message written over and over is one entry. It counts
// consecutive repeats alone, so a second message written between them defeats it: a fatal refusal
// that also announced an unloading wrote two lines per event and collapsed to neither. What this
// rules out is that second line, which is what leaves one standing refusal reading as one thing.
TEST_CASE("a_refusal_of_an_edited_value_repeated_writes_that_one_message_and_nothing_between_the_repeats")
{
    const std::string reported = reported_over(6, motion_ops{.task_space_pose = &ill_formed});

    CHECK(occurrences(reported, "'motion.task_space_pose' refused the request") == 6u);
    CHECK(occurrences(reported, "is asked to unload") == 0u);
}

// Each bearable kind is asserted on its own, so reclassifying one of them cannot pass on the
// strength of the other. The configuration is compared exactly, against what the renderer read back
// rather than against what was written to it: the renderer holds single precision, and a tolerance
// loose enough to absorb that round trip would also absorb a substituted value.
TEST_CASE("an_unbound_binding_leaves_the_composition_loaded_and_the_configuration_exactly_as_it_was")
{
    const outcome after = previewing(motion_ops{});

    CHECK(after.asked_to_unload == 0);
    CHECK(after.configuration_untouched);
}

TEST_CASE("an_answer_that_does_not_exist_leaves_the_composition_loaded_and_the_configuration_exactly_as_it_was")
{
    const outcome after = previewing(motion_ops{.task_space_pose = &unreachable});

    CHECK(after.asked_to_unload == 0);
    CHECK(after.configuration_untouched);
}

// A controller composed with no route to say it cannot continue is a state rather than a defect: it
// reports the refusal it received and decides nothing.
TEST_CASE("a_fatal_refusal_with_no_route_to_ask_through_reports_and_does_not_fault")
{
    scene_robot driven          = two_joint_arm(robot_ops{});
    robot_controller controller = controlling(driven, motion_ops{.task_space_pose = &unsupported}, nullptr);

    CHECK_NOTHROW(controller.preview_task_space_pose(transform::Identity()));
}

// The driven chain carries two joints, so a three-element configuration is a size the binding does
// not serve and is refused rather than assigned.
TEST_CASE("a_configuration_of_a_length_the_driven_chain_does_not_carry_is_refused_and_unloads_the_composition")
{
    const outcome after = previewing_configuration(three_joints());

    CHECK(after.asked_to_unload == 1);
    CHECK(after.configuration_untouched);
}

// A waypoint count a factory does not serve leaves the composition able to answer for itself: the
// arm is where it was, every window still reads it, and the next command still runs. So the count is
// a refused request rather than a reason to unload, on either of the two waypoint routes.
TEST_CASE("a_configuration_waypoint_count_the_shipped_factory_refuses_leaves_the_composition_loaded")
{
    const std::vector<joint_vector> three{configuration(0.1, 0.2), configuration(0.3, 0.4), configuration(0.5, 0.6)};
    const outcome after = commanding_configurations(three);

    CHECK(after.asked_to_unload == 0);
    CHECK(after.configuration_untouched);
}

TEST_CASE("a_pose_waypoint_count_the_shipped_factory_refuses_leaves_the_composition_loaded")
{
    const std::vector<transform> none;
    const outcome after = commanding_poses(none);

    CHECK(after.asked_to_unload == 0);
    CHECK(after.configuration_untouched);
}
