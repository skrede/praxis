#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_OPENED_RUNG_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_OPENED_RUNG_H

#include "labeled_panels.h"
#include "composed_panels.h"

#include "panel_keys.h"
#include "imgui_frame.h"

#include "praxis/presets/euler_rungs.h"

#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/frame_window.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/frame_selector_window.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/math/Vector3.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/core/Object3D.hpp>

#include <memory>
#include <cstddef>
#include <functional>

namespace praxis::fixture {

// One rung standing where a loaded composition would have left it: the stencil in a scene of its
// own, and every window having applied what it opened at.
struct opened_rung
{
    explicit opened_rung(presets::euler_rung which)
            : stage()
            , composed(presets::euler_rung_preset(unwired(stage), rigid_motion::baseline(), which))
    {
        REQUIRE(composed != nullptr);
        REQUIRE(composed->initialize().has_value());
        for(const std::shared_ptr<scene::imgui_window> &window : composed->windows)
            window->initialize();
    }

    opened_rung(const opened_rung &) = delete;

    rigid_motion::frame_stencil &body() const
    {
        return static_cast<rigid_motion::frame_stencil &>(*composed->stencil);
    }

    rigid_motion::frame_selector_window &selector() const
    {
        const auto held = std::dynamic_pointer_cast<rigid_motion::frame_selector_window>(composed->windows.front());
        REQUIRE(held != nullptr);

        return *held;
    }

    rigid_motion::frame_window &panel() const
    {
        const auto held = std::dynamic_pointer_cast<rigid_motion::frame_window>(composed->windows[1]);
        REQUIRE(held != nullptr);

        return *held;
    }

    // Declared ahead of the composition so that it is destroyed after one: the stencil holds this
    // scene for as long as the composition does.
    threepp::Scene stage;
    std::shared_ptr<scene::preset> composed;
};

// Where the mark stands in the renderer's world, which is the frame the extent it is taken into is
// expressed in. It composes its own placement inside the world-matrix update the renderer gives every
// node, and reading a node's placement does not go through that update, so the scene is brought up to
// date first.
inline threepp::Vector3 marked_place(opened_rung &rung)
{
    rung.body().render();
    rung.stage.updateMatrixWorld(true);

    threepp::Object3D *const drawn = rung.stage.getObjectByName("mark");
    REQUIRE(drawn != nullptr);
    REQUIRE(drawn->visible);

    threepp::Vector3 put;
    drawn->getWorldPosition(put);

    return put;
}

// A control set written here rather than composed, so what a rung's panel draws is read against a
// panel over the same stencil and the same opening differing in one control.
inline rigid_motion::frame_window::controls asking(bool parent, std::size_t first)
{
    rigid_motion::frame_window::controls asked;
    asked.parent                = parent;
    asked.visibility            = false;
    asked.first_panelled_object = first;

    return asked;
}

// A panel written here stands behind a selection as a composed one does, so what a case reads across
// the two is one control's difference rather than one panel against every panel.
inline std::function<std::size_t()> about(std::size_t index)
{
    return [index] { return index; };
}

inline std::size_t items_of(rigid_motion::frame_window &panel)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    return navigable_items(frames, [&panel] { panel.render(); });
}

inline void drive_panel(rigid_motion::frame_window &panel)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    drive_every_control(frames, [&panel] { panel.render(); });
}

}

#endif
