#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/compose_arm.h"
#include "praxis/manipulator/scene_robot.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/robot_controller.h"
#include "praxis/manipulator/scene_robot_builder.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/extension/coverage.h"
#include "praxis/extension/held_handle.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include <spdlog/spdlog.h>

#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <utility>
#include <optional>

namespace praxis::manipulator {

namespace {

const char *refusal_name(refusal reason)
{
    switch(reason)
    {
        case refusal::unsupported_input:
            return "unsupported input";
        case refusal::degenerate:
            return "degenerate";
        case refusal::no_solution:
            return "no solution";
        case refusal::not_implemented:
            return "not implemented";
    }

    return "unclassified";
}

std::shared_ptr<scene::preset> report_refusal(const char *builder, const std::string &model, refusal reason)
{
    spdlog::error("praxis: '{}' refused the description of model '{}' as {}; the preset is not composed", builder, model, refusal_name(reason));

    return nullptr;
}

// A capability refusing below the scene graph still leaves an arm to draw, so what is reported is
// what the preset answers instead. It is reported here, once per composition, because a resolution
// asked per frame or per sample refuses silently.
unexpected<refusal> undrivable(const char *builder, const std::string &model, refusal reason)
{
    spdlog::error("praxis: '{}' refused the description of model '{}' as {}; the arm is shown and not driven", builder, model, refusal_name(reason));

    return unexpected(reason);
}

// The three refusals a description can earn once it has a scene graph. None of them is fatal: each
// is the capability that produced it saying so, and binding one is never the price of binding
// another.
expected<scene_robot, refusal> driven_arm(const meios::model<> &description, const threepp::Robot &handle, const capabilities &arm, const rigid_motion::screw_ops &screw,
                                          const rigid_motion::frame_ops &frames)
{
    auto chain = arm.modeling.build_chain(description);
    if(!chain)
        return undrivable("manipulator.modeling.build_chain", description.name, chain.error());

    auto solver = kinematics::compose(std::move(*chain), arm.fk, arm.dk, arm.ik, screw, frames);
    if(!solver)
        return undrivable("manipulator.kinematics.compose", description.name, solver.error());

    // The composition below names both counts where it compares them, so the disagreement is not
    // reported a second time here.
    auto composed = scene_robot::compose(std::move(*solver), arm.robot, frames, static_cast<std::uint32_t>(handle.numDOF()));
    if(!composed)
        return unexpected(composed.error());

    return composed;
}

// Every field derived from a solver that does not exist carries the refusal that stopped the
// derivation; the configuration and the tool offset are what a reader can still be given.
arm_snapshot undriven_snapshot(const joint_vector &initial, refusal reason)
{
    return arm_snapshot{initial,
                        joint_limits{},
                        transform::Identity(),
                        unexpected(reason),
                        unexpected(reason),
                        unexpected(reason),
                        unexpected(reason),
                        unexpected(reason),
                        unexpected(reason),
                        recording_parameters{},
                        0.0,
                        false,
                        scheduler::task_counters{},
                        {},
                        unexpected(reason),
                        unexpected(reason),
                        jacobian_manipulability{unexpected(reason), unexpected(reason)},
                        jacobian_manipulability{unexpected(reason), unexpected(reason)},
                        {},
                        {},
                        nullptr,
                        nullptr,
                        time_scaling_choice{},
                        std::nullopt};
}

robot_slot_set defaulted_robot_slots(const robot_ops &ops)
{
    const capability_view described = view_of(ops);
    robot_slot_set held;

    for(std::uint32_t index = 0; index < static_cast<std::uint32_t>(robot_slot::count); ++index)
        if(holds_default(described, index))
            held.set(static_cast<robot_slot>(index));

    return held;
}

rigid_motion::screw_slot_set defaulted_screw_slots(const rigid_motion::screw_ops &ops)
{
    const capability_view described = view_of(ops);
    rigid_motion::screw_slot_set held;

    for(std::uint32_t index = 0; index < static_cast<std::uint32_t>(rigid_motion::screw_slot::count); ++index)
        if(holds_default(described, index))
            held.set(static_cast<rigid_motion::screw_slot>(index));

    return held;
}

// The publication is made once, at the composition, and every frame reads that one: with no solver
// there is nothing below the stencil left to ask. No window is composed either, because a window is
// built from a gated arm state that does not exist here.
std::shared_ptr<scene::preset> undriven_preset(const scene::preset_site &site, const std::shared_ptr<threepp::Robot> &handle, attached_models attached, const joint_vector &initial,
                                               const rigid_motion::capabilities &motions, refusal reason)
{
    auto published = std::make_shared<arm_publisher>();
    published->publish(std::make_shared<const arm_snapshot>(undriven_snapshot(initial, reason)));

    auto stencil =
            std::make_shared<loadable_robot_stencil>(handle, std::move(attached), site.scene, site.render, published->reader(), motions.screw, defaulted_screw_slots(motions.screw));
    auto composed = std::make_shared<scene::preset>(stencil, std::vector<std::shared_ptr<scene::imgui_window>>{}, site.add_window, site.remove_window);

    composed->work       = site.work;
    composed->release_cb = [published]() mutable { published.reset(); };

    return composed;
}

void release_arm(std::shared_ptr<owned_arm> &owned, std::shared_ptr<robot_controller> &control, bool concluding)
{
    if(concluding)
        static_cast<void>(control->conclude_recording());
    control.reset();
    owned.reset();
}

// The aggregate is built where it stands, because the gate is neither copyable nor movable, and its
// last share is handed to the preset's release callables: freeing it anywhere earlier would let a
// handler the gate has already posted run against storage that is gone. It publishes its first
// snapshot in its own constructor, so every reader handed out below it reads a published value.
std::shared_ptr<scene::preset> composed_preset(const scene::preset_site &site, const std::shared_ptr<threepp::Robot> &handle, attached_models attached,
                                               const std::shared_ptr<scene_robot> &robot, const std::shared_ptr<robot_controller> &controller, const rigid_motion::capabilities &motions,
                                               forward_kinematics_ops forward, differential_kinematics_ops differential, trajectory::path_ops path, robot_slot_set inert,
                                               const arm_window_composer &windows)
{
    const rigid_motion::screw_slot_set unbound = defaulted_screw_slots(motions.screw);

    auto published = std::make_shared<arm_publisher>();
    auto owned     = std::make_shared<owned_arm>(site.work, site.work, robot, controller, published);
    auto stencil   = std::make_shared<loadable_robot_stencil>(handle, std::move(attached), site.scene, site.render, published->reader(), motions.screw, unbound);

    const arm_window_inputs built{*stencil, published->reader(), owned, motions.frame, inert, unbound, robot->solver().space_chain(), motions.screw, forward, differential, path};

    auto composed  = std::make_shared<scene::preset>(stencil, windows(built), site.add_window, site.remove_window);
    composed->work = site.work;
    // The recording is concluded here rather than anywhere earlier: this runs after the last handler
    // the strand admitted and before the arm is freed, which is where a write that has to be
    // reported belongs.
    composed->release_cb = [owned, control = controller]() mutable { release_arm(owned, control, true); };
    // The same conclusion where no acknowledgment will run: the flag is the refusal of the strand's
    // retirement, which is what destroys the callable above uncalled and takes its shares with it.
    composed->release_fallback_cb = [owned, control = controller](bool unacknowledged) mutable { release_arm(owned, control, unacknowledged); };

    return composed;
}

// An empty opening configuration is a scenario with no opinion about where the arm starts, rather
// than a scenario naming zero joints. A length that is neither is answered with nothing: a
// configuration padded or clamped to fit would open the arm at joint values no scenario named.
std::optional<joint_vector> opening_configuration(const std::string &model, const joint_vector &supplied, std::uint32_t joints)
{
    const auto held = static_cast<Eigen::Index>(joints);
    if(supplied.size() == held)
        return supplied;

    if(supplied.size() == 0)
        return joint_vector::Zero(held);

    spdlog::error("praxis: 'manipulator.compose_arm' refused the description of model '{}' as {}; its opening configuration has {} joint values and the arm has {} joints, so the "
                  "preset is not composed",
                  model, refusal_name(refusal::unsupported_input), supplied.size(), joints);

    return std::nullopt;
}

}

// One parsed description feeds both the chain derivation and the scene graph, which is what makes
// the screw axes and the rendered links agree on the frame they are expressed in. The scene graph is
// the one derivation with nothing to fall back on: with no body there is no preset.
std::shared_ptr<scene::preset> compose_arm(const meios::model<> &description, const scene::preset_site &site, attached_models attached, const capabilities &arm,
                                           const trajectory::capabilities &shapes, const rigid_motion::capabilities &motions, const joint_vector &initial,
                                           const arm_window_composer &windows)
{
    auto handle = build_scene_robot(description);
    if(!handle)
        return report_refusal("manipulator.build_scene_robot", description.name, handle.error());

    auto driven = driven_arm(description, held(*handle, "the composed arm", "renderer handle"), arm, motions.screw, motions.frame);
    if(!driven)
        return undriven_preset(site, *handle, std::move(attached), initial, motions, driven.error());

    auto robot                                = std::make_shared<scene_robot>(std::move(*driven));
    const std::optional<joint_vector> opening = opening_configuration(description.name, initial, robot->joint_count());
    if(!opening)
        return nullptr;

    robot->set_joint_positions(*opening);

    auto controller =
            std::make_shared<robot_controller>(*robot, arm.motion, shapes.path, arm.trajectory, shapes.time_scaling, shapes.trajectory, motions.screw, site.ask_unload, site.root);

    return composed_preset(site, *handle, std::move(attached), robot, controller, motions, arm.fk, arm.dk, shapes.path, defaulted_robot_slots(arm.robot), windows);
}

}
