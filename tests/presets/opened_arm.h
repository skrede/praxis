#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_OPENED_ARM_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_OPENED_ARM_H

#include "drawn_lines.h"
#include "described_arm.h"
#include "composed_panels.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/screw_chain.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <meios/urdf/load.h>

#include <threepp/scenes/Scene.hpp>

#include <threepp/core/Object3D.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <filesystem>

// The descriptions are deployed beside the demonstration executable, so a configure without the
// demonstration leaves nothing to parse; that is a valid configuration and skips.
#ifndef PRAXIS_DEMO_RESOURCE_DIR
    #define PRAXIS_DEMO_RESOURCE_DIR ""
#endif

namespace praxis::fixture {

// What a point read back out of a float buffer and turned out of the renderer's world is comparable
// at, in metres.
inline constexpr double read_back = 1.0e-4;

// The universal machine's description is one document parameterized over a whole product line, so
// which arm it describes is named by argument rather than by path.
inline std::optional<presets::arm_scenario> deployed_machine(const char *path, bool parameterized)
{
    const std::filesystem::path root{PRAXIS_DEMO_RESOURCE_DIR};
    if(root.empty() || !std::filesystem::exists(root / path))
        return std::nullopt;

    presets::arm_scenario chosen;
    chosen.description = root / path;
    chosen.options.package_roots.push_back(root);
    if(parameterized)
    {
        chosen.options.args["ur_type"] = "ur3e";
        chosen.options.args["name"]    = "ur3e";
    }

    return chosen;
}

// The chain the reference derivation reads out of the same description, derived here rather than
// taken from the composition, so a case comparing against it compares against the description.
inline manipulator::screw_chain derived_chain(const std::filesystem::path &description)
{
    const auto loaded = meios::load(description, meios::load_options{});
    REQUIRE(loaded.has_value());

    const expected<manipulator::screw_chain, refusal> chain = manipulator::baseline().modeling.build_chain(loaded->robot);
    REQUIRE(chain.has_value());

    return *chain;
}

// A point lies on the axis a screw names exactly where the screw's own linear part is what the
// angular part crossed into that point gives up, which is the relation the axis is defined by.
inline bool on_axis(const screw_axis &named, const Eigen::Vector3d &at)
{
    return (named.head<3>().cross(at) + named.tail<3>()).norm() < read_back;
}

// One drawn axis per joint, counted by the name the stencil carries each of them under and stopping
// at the first name the scene does not hold, so a decoration wider than the arm reads here too.
inline std::size_t drawn_axes(threepp::Scene &target)
{
    std::size_t counted = 0;
    while(first_line_under(target, manipulator::loadable_robot_stencil::joint_axis_name(counted)) != nullptr)
        ++counted;

    return counted;
}

// One headless scene an arm scenario is opened against, with a real strand beneath it because
// placing the decoration runs on the strand the frames are drawn on.
struct opened_arm
{
    opened_arm()
            : loop(scheduler::inline_workers)
            , scene(threepp::Scene::create())
    {
    }

    scene::preset_site site()
    {
        const scene::window_route nowhere = [](const std::shared_ptr<scene::imgui_window> &) {};

        return scene::preset_site{*scene, loop.main_strand(), *loop.make_strand(), [] {}, opening_route(), nowhere, {}};
    }

    std::size_t descendants() const
    {
        std::size_t counted = 0;
        scene->traverse([&counted](threepp::Object3D &) { ++counted; });

        return counted;
    }

    std::shared_ptr<scene::preset> open(const presets::arm_scenario &chosen, const manipulator::arm_composition &opened)
    {
        std::shared_ptr<scene::preset> composed = presets::arm_preset(site(), manipulator::baseline(), trajectory::baseline(), rigid_motion::baseline(), chosen, opened);
        if(composed != nullptr)
            REQUIRE(composed->initialize().has_value());

        return composed;
    }

    void draw(const scene::preset &composed)
    {
        REQUIRE(loop.main_strand().post([&composed] { composed.stencil->render(); }).has_value());
        REQUIRE(loop.drain().has_value());
        scene->updateMatrixWorld(true);
    }

    void release(std::shared_ptr<scene::preset> composed)
    {
        composed->tear_down();
        REQUIRE(loop.retire_strand(composed->work, std::move(composed->release_cb)).has_value());
        REQUIRE(loop.drain().has_value());
    }

    scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
};

}

#endif
