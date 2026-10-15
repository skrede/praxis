#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_COMPOSED_MOTION_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_COMPOSED_MOTION_H

#include "drawn_lines.h"
#include "described_arm.h"
#include "composed_panels.h"
#include "bound_trajectories.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/motion_drawings.h"
#include "praxis/manipulator/loadable_robot_stencil.h"
#include "praxis/manipulator/path_comparison_window.h"
#include "praxis/manipulator/trajectory_preview_window.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/plot_window.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <array>
#include <memory>
#include <string>
#include <cstddef>
#include <utility>
#include <algorithm>

namespace praxis::fixture {

inline constexpr std::size_t preview_ask_control = 0;
inline constexpr std::size_t scaling_control     = 1;

// The trapezoid stands one entry below the quintic a scenario opens on.
inline constexpr std::size_t trapezoid_entry = 1;

// A composer, the panel its opening motion is read off, whether that motion has to be asked for, and
// the lines it is drawn as.
struct trajectory_composer
{
    const char *named;
    manipulator::arm_composition (*compose)(presets::arm_scenario);
    const char *panel;
    bool asked;
    std::array<const char *, 3> lines;
};

using compared_paths = manipulator::path_comparison_window;

inline constexpr trajectory_composer point_to_point{"point to point", &presets::arm_windows_point_to_point, "Preview", true, {manipulator::commanded_motion_path}};
inline constexpr trajectory_composer via_point{"via point", &presets::arm_windows_via_point, "Preview", true, {manipulator::commanded_motion_path}};
inline constexpr trajectory_composer path_comparison{
        "path comparison", &presets::arm_windows_path_comparison, "Comparison", false, {compared_paths::joint_space_path, compared_paths::decoupled_path, compared_paths::screw_path}};
inline constexpr std::array trajectory_composers{point_to_point, via_point, path_comparison};

// One headless scene a composer is opened against through the preset entry point, over the trajectory
// aggregate a case hands it and the reference of every other capability.
struct composed_motion
{
    composed_motion(const trajectory::capabilities &bound, const trajectory_composer &one)
            : described(6, "six")
            , unloaded(false)
            , loop(scheduler::inline_workers)
            , scene(threepp::Scene::create())
            , opened(one)
    {
        const presets::arm_scenario chosen = described_by(described.where);
        const scene::window_route nowhere  = [](const std::shared_ptr<scene::imgui_window> &) {};
        const scene::preset_site site{*scene, loop.main_strand(), *loop.make_strand(), [this](std::string) { unloaded = true; }, opening_route(), nowhere, {}};

        composed = presets::arm_preset(site, manipulator::baseline(), bound, rigid_motion::baseline(), chosen, one.compose(chosen));
        REQUIRE(composed != nullptr);
        REQUIRE(composed->initialize().has_value());
    }

    composed_motion(const composed_motion &) = delete;

    std::shared_ptr<scene::imgui_window> panel(const std::string &named) const
    {
        for(const std::shared_ptr<scene::imgui_window> &held : composed->windows)
            if(held->display_name() == named)
                return held;

        FAIL("the composition opened no panel named " + named);

        return nullptr;
    }

    // The lines the opening motion is drawn as, once the arm has answered the ask and the panel has
    // told the drawing what it published.
    drawn_lines opening()
    {
        if(opened.asked)
            press_at(*panel(opened.panel), preview_ask_control);
        REQUIRE(loop.drain().has_value());
        static_cast<void>(panels_opened_by(*panel(opened.panel)));
        REQUIRE(loop.main_strand().post([this] { composed->stencil->render(); }).has_value());
        REQUIRE(loop.drain().has_value());
        scene->updateMatrixWorld(true);

        drawn_lines read;
        for(const char *line : opened.lines)
            if(line != nullptr)
                read.push_back(line_in_world(*scene, manipulator::loadable_robot_stencil::pose_path_name(line)));

        return read;
    }

    void choose_the_trapezoid()
    {
        REQUIRE(loop.drain().has_value());
        take_entry_at(*panel("Control parameters"), scaling_control, trapezoid_entry);
    }

    scene::plot_reading previewed() const
    {
        const auto shown = std::dynamic_pointer_cast<manipulator::trajectory_preview_window>(panel("Preview"));
        REQUIRE(shown != nullptr);

        return shown->reading();
    }

    // The name of the curve the preview marks as the one the arm is set to.
    std::string chosen() const
    {
        const scene::plot_reading shown = previewed();
        REQUIRE_FALSE(shown.frames.empty());
        for(const scene::plot_series &curve : shown.frames.front().series)
            if(curve.name.find("(chosen)") != std::string::npos)
                return curve.name;

        return {};
    }

    ~composed_motion()
    {
        composed->tear_down();
        REQUIRE(loop.retire_strand(composed->work, std::move(composed->release_cb)).has_value());
        REQUIRE(loop.drain().has_value());
    }

    described_arm described;
    bool unloaded;
    scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    trajectory_composer opened;
    std::shared_ptr<scene::preset> composed;
};

inline bool drawn_whole(const drawn_lines &read)
{
    return !read.empty() && std::all_of(read.begin(), read.end(), [](const auto &line) { return line.size() > 1u; });
}

}

#endif
