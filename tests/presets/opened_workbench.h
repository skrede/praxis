#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_OPENED_WORKBENCH_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_OPENED_WORKBENCH_H

#include "composed_panels.h"
#include "labeled_panels.h"

#include "panel_keys.h"
#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/presets/frame_workbench.h"

#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/frame_window.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/frame_roster_window.h"
#include "praxis/rigid_motion/frame_selector_window.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/math/Vector3.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/core/Object3D.hpp>

#include <memory>
#include <string>
#include <cstddef>
#include <ostream>

namespace praxis::fixture {

inline constexpr std::size_t opened_with = 2;

// One workbench standing where a loaded composition would have left it, over a scene of its own.
struct opened_workbench
{
    opened_workbench()
            : stage()
            , composed(presets::frame_workbench_preset(unwired(stage), rigid_motion::baseline()))
    {
        REQUIRE(composed != nullptr);
        REQUIRE(composed->initialize().has_value());
        for(const std::shared_ptr<scene::imgui_window> &window : composed->windows)
            window->initialize();
    }

    opened_workbench(const opened_workbench &) = delete;

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

    rigid_motion::frame_roster_window &roster() const
    {
        const auto held = std::dynamic_pointer_cast<rigid_motion::frame_roster_window>(composed->windows[2]);
        REQUIRE(held != nullptr);

        return *held;
    }

    scene::imgui_window &rotation() const
    {
        return *composed->windows[3];
    }

    scene::imgui_window &transformation() const
    {
        return *composed->windows[4];
    }

    // Declared ahead of the composition so that it is destroyed after one: the stencil holds this
    // scene for as long as the composition does.
    threepp::Scene stage;
    std::shared_ptr<scene::preset> composed;
};

// A roster row carries the name of the frame it stands for, drawn inside the identifier scope that
// row's own index pushes. The roster draws its rows before anything else, so what the keyboard walk
// stands on first is the row of the frame at that index and nothing is drawn above it.
inline void first_control_is(rigid_motion::frame_roster_window &roster, std::size_t scope, const char *named)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing over = [&roster] { roster.render(); };
    stand_below_top(frames, over, 0);

    INFO("the roster's first control is not the row named " << named);
    REQUIRE(standing_on() == control_id(roster.display_name().c_str(), scope, named));
}

// A pose turned as well as moved, so a readout standing on the object it is given to stands
// somewhere it did not stand before. The turn is composed through the reference implementation,
// because the inert one this repository also ships answers the identity to every angle.
inline transform turned_and_moved()
{
    transform put         = transform::Identity();
    put.block<3, 3>(0, 0) = rigid_motion::baseline().frame.rotate_z(to_radians(35.0));
    put(0, 3)             = 1.25;

    return put;
}

// Where the mark stands in the renderer's world, which is the frame the extent it is taken into is
// expressed in. It composes its own placement inside the world-matrix update the renderer gives every
// node, and reading a node's placement does not go through that update, so the scene is brought up to
// date first.
inline threepp::Vector3 marked_place(opened_workbench &built)
{
    built.body().render();
    built.stage.updateMatrixWorld(true);

    threepp::Object3D *const drawn = built.stage.getObjectByName("mark");
    REQUIRE(drawn != nullptr);
    REQUIRE(drawn->visible);

    threepp::Vector3 put;
    drawn->getWorldPosition(put);

    return put;
}

// What the parameters panel, the two readouts, the scene and the axis tick say, taken together: they
// are five readers of one choice, and what says each of them follows it is that all five move on it
// at once.
struct read_by_selection
{
    explicit read_by_selection(opened_workbench &built)
            : axes(built.body().axes_shown(built.selector().selected_object()))
            , panel(geometry_of(built.panel()))
            , rotation(geometry_of(built.rotation()))
            , transformation(geometry_of(built.transformation()))
            , mark(marked_place(built))
    {
    }

    bool operator==(const read_by_selection &) const = default;

    bool axes;
    std::size_t panel;
    std::size_t rotation;
    std::size_t transformation;
    threepp::Vector3 mark;
};

// Written out so that a failure names which of the five did not stand where it was expected to.
inline std::ostream &operator<<(std::ostream &to, const read_by_selection &read)
{
    return to << "axes " << read.axes << ", panel " << read.panel << ", rotation " << read.rotation << ", transformation " << read.transformation << ", mark " << read.mark;
}

inline std::size_t descendants(threepp::Scene &of)
{
    std::size_t counted = 0;
    of.traverse([&counted](threepp::Object3D &) { ++counted; });

    return counted;
}

}

#endif
