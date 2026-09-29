#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_MODELS_WITHOUT_WINDOWS_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_MODELS_WITHOUT_WINDOWS_H

#include "opened_arm.h"
#include "carried_models.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/compose_arm.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/axis_order.h"
#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/core/Object3D.hpp>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>
#include <filesystem>

namespace praxis::fixture {

// The composition with the windows titled in `dropped` taken out of the list it returns, after it
// has built them.
inline manipulator::arm_composition leaving_out(manipulator::arm_composition composed, std::vector<std::string> dropped)
{
    composed.windows = [inner = std::move(composed.windows), dropped = std::move(dropped)](const manipulator::arm_window_inputs &built)
    {
        std::vector<std::shared_ptr<scene::imgui_window>> opened = inner(built);
        std::erase_if(opened, [&dropped](const std::shared_ptr<scene::imgui_window> &panel) { return std::ranges::find(dropped, panel->display_name()) != dropped.end(); });

        return opened;
    };

    return composed;
}

// Both models placed off where an unplaced model stands: turned, moved and scaled, with a tool offset
// that is turned and moved as well.
inline presets::arm_scenario placing_models(const std::filesystem::path &description, const std::filesystem::path &tool, const std::filesystem::path &world, bool active)
{
    presets::arm_scenario chosen = carrying_models(description, tool, world);
    chosen.tool         = manipulator::tool_window::settings(active, tool.string(), manipulator::tool_window::tool_view::kinematics_transform, Eigen::Vector3f(10.f, 20.f, 30.f),
                                                             axis_order::zyx, Eigen::Vector3f(1.5f, 1.5f, 1.5f), Eigen::Vector3f(0.05f, -0.02f, 0.04f), Eigen::Vector3f(0.f, 15.f, 0.f),
                                                             axis_order::zyx, Eigen::Vector3f(0.1f, 0.2f, 0.3f));
    chosen.world_object = manipulator::world_object_window::settings(active, world.string(), manipulator::world_object_window::world_view::transform, Eigen::Vector3f(2.f, 2.f, 2.f),
                                                                     Eigen::Vector3f(0.4f, -0.3f, 0.2f), Eigen::Vector3f(15.f, 25.f, 35.f));

    return chosen;
}

inline std::shared_ptr<scene::preset> opened_under(opened_arm &stage, const presets::arm_scenario &chosen, const manipulator::arm_composition &opened,
                                                   const rigid_motion::capabilities &motions)
{
    std::shared_ptr<scene::preset> composed = presets::arm_preset(stage.site(), manipulator::baseline(), trajectory::baseline(), motions, chosen, opened);
    REQUIRE(composed != nullptr);
    REQUIRE(composed->initialize().has_value());
    REQUIRE(stage.loop.drain().has_value());
    stage.draw(*composed);

    return composed;
}

inline manipulator::loadable_robot_stencil &stencil_of(const scene::preset &composed)
{
    const auto stencil = std::dynamic_pointer_cast<manipulator::loadable_robot_stencil>(composed.stencil);
    REQUIRE(stencil != nullptr);

    return *stencil;
}

inline Eigen::Vector3d placed_at(threepp::Object3D &node)
{
    threepp::Vector3 at;
    node.getWorldPosition(at);

    return Eigen::Vector3d{at.x, at.y, at.z};
}

inline double largest_difference(const threepp::Matrix4 &first, const threepp::Matrix4 &second)
{
    double largest = 0.0;
    for(std::size_t index = 0; index < first.elements.size(); ++index)
        largest = std::max(largest, static_cast<double>(std::abs(first.elements[index] - second.elements[index])));

    return largest;
}

inline double marker_separation(const scene::preset &composed)
{
    manipulator::loadable_robot_stencil &stencil = stencil_of(composed);

    return (placed_at(*stencil.attached_at(manipulator::flange_attachment::tool_frame_marker)) - placed_at(*stencil.attached_at(manipulator::flange_attachment::frame_marker))).norm();
}

}

#endif
