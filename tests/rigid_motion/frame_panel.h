#ifndef HPP_GUARD_PRAXIS_TESTS_RIGID_MOTION_FRAME_PANEL_H
#define HPP_GUARD_PRAXIS_TESTS_RIGID_MOTION_FRAME_PANEL_H

#include "panel_keys.h"

#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/frame_window.h"
#include "praxis/rigid_motion/frame_stencil.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <imgui.h>

#include <Eigen/Core>

#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <functional>

namespace praxis::fixture {

inline std::vector<rigid_motion::stencil_object> three_objects()
{
    rigid_motion::object_body bare;
    bare.shape = rigid_motion::body_shape::none;

    return {rigid_motion::stencil_object{"First", rigid_motion::axes_settings{}, rigid_motion::object_body{}},
            rigid_motion::stencil_object{"Second", rigid_motion::axes_settings{}, bare}, rigid_motion::stencil_object{"Third", rigid_motion::axes_settings{}, bare}};
}

inline rigid_motion::frame_window::placement placed(float along, std::optional<std::size_t> parent)
{
    rigid_motion::frame_window::placement one;
    one.position = Eigen::Vector3f{along, 0.f, 0.f};
    one.parent   = parent;

    return one;
}

// The last object hangs under the middle one, so a parent relation the panel must not be able to
// move stands whichever objects a composition offers a panel for.
inline rigid_motion::frame_window::settings resting()
{
    rigid_motion::frame_window::settings chosen;
    chosen.objects = {placed(0.f, std::nullopt), placed(0.4f, std::nullopt), placed(0.8f, std::size_t{1})};

    return chosen;
}

// One stencil and one window over it, standing where a composition would have left the two. Each
// constructor reaches the window through the window's own constructors rather than through one
// another, so a composition that names no control set is composed the way a scenario without one
// composes it.
struct staged
{
    staged()
            : scene()
            , body(scene, three_objects(), rigid_motion::frame_ops{}, rigid_motion::fixed_frame{"Ground", rigid_motion::axes_settings{}})
            , panel("Frame", body, rigid_motion::frame_ops{}, resting())
    {
        REQUIRE(body.initialize().has_value());
        panel.initialize();
    }

    explicit staged(const rigid_motion::frame_window::controls &offered)
            : scene()
            , body(scene, three_objects(), rigid_motion::frame_ops{}, rigid_motion::fixed_frame{"Ground", rigid_motion::axes_settings{}})
            , panel("Frame", body, rigid_motion::frame_ops{}, resting(), std::string(), offered)
    {
        REQUIRE(body.initialize().has_value());
        panel.initialize();
    }

    // A window handed a selection source draws the panel that source names and no other, so a case
    // moves the panel by moving what the source reads rather than by reaching a control.
    staged(const rigid_motion::frame_window::controls &offered, std::function<std::size_t()> selecting)
            : scene()
            , body(scene, three_objects(), rigid_motion::frame_ops{}, rigid_motion::fixed_frame{"Ground", rigid_motion::axes_settings{}})
            , panel("Frame", body, rigid_motion::frame_ops{}, resting(), std::string(), offered, std::move(selecting))
    {
        REQUIRE(body.initialize().has_value());
        panel.initialize();
    }

    staged(const staged &) = delete;

    threepp::Scene scene;
    rigid_motion::frame_stencil body;
    rigid_motion::frame_window panel;
};

inline drawing over(rigid_motion::frame_window &panel)
{
    return [&panel] { panel.render(); };
}

// Every vertex one drawing produced, so two panels are compared by what they put on screen rather
// than by how much of it there was.
inline std::size_t geometry_of(rigid_motion::frame_window &panel)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);
    frames.draw(over(panel));

    REQUIRE(frames.has_draw_data());
    REQUIRE(frames.vertices() > 0);

    return frames.signature();
}

inline std::size_t items_offered(rigid_motion::frame_window &panel)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    return navigable_items(frames, over(panel));
}

inline void drive(rigid_motion::frame_window &panel)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    drive_every_control(frames, over(panel));
}

inline void press_on(rigid_motion::frame_window &panel, const char *label)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing draw = over(panel);
    press_on(frames, draw, panel.display_name().c_str(), label);
}

// A control an object's own panel draws is drawn inside the identifier scope that panel pushes, so
// it is named by the object it belongs to as well as by its label.
inline void press_on(rigid_motion::frame_window &panel, std::size_t scope, const char *label)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing draw = over(panel);
    press_on(frames, draw, panel.display_name().c_str(), scope, label);
}

inline void take_entry_on(rigid_motion::frame_window &panel, std::size_t scope, const char *label)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing draw = over(panel);
    take_entry_on(frames, draw, panel.display_name().c_str(), scope, label);
}

}

#endif
