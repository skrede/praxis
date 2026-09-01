#ifndef HPP_GUARD_PRAXIS_TESTS_RIGID_MOTION_FRAME_WINDOW_STAGE_H
#define HPP_GUARD_PRAXIS_TESTS_RIGID_MOTION_FRAME_WINDOW_STAGE_H

#include "panel_keys.h"
#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/capabilities.h"
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

namespace praxis::fixture {

// Every opening placement carries a turn about the first axis of its order, so a readout that
// dropped a rotation and kept a translation could not pass a case below.
inline rigid_motion::frame_window::placement turned(float along, float turn_degrees, std::optional<std::size_t> parent)
{
    return rigid_motion::frame_window::placement{axis_order::zyx, Eigen::Vector3f{along, 0.2f, 0.5f}, Eigen::Vector3f{turn_degrees, 0.f, 0.f}, parent};
}

inline std::vector<rigid_motion::stencil_object> four_objects()
{
    std::vector<rigid_motion::stencil_object> drawn;
    for(const char *named : {"Fixed", "One", "Two", "Three"})
        drawn.push_back(rigid_motion::stencil_object{named, rigid_motion::axes_settings{}, rigid_motion::object_body{}});

    return drawn;
}

// The last object hangs under the one before it, so a compaction the panel must follow stands
// whichever object a case takes out.
inline rigid_motion::frame_window::settings arranged()
{
    return rigid_motion::frame_window::settings{
            {turned(0.f, 25.f, std::nullopt), turned(0.4f, -40.f, std::nullopt), turned(0.8f, 65.f, std::nullopt), turned(1.2f, 110.f, std::size_t{2})}};
}

// The bound operations are the reference ones: a panel decomposed through an inert binding would
// read the identity back for every pose and prove nothing.
struct moving_set
{
    explicit moving_set(const rigid_motion::frame_window::controls &offered)
            : scene()
            , body(scene, four_objects(), rigid_motion::baseline().frame, rigid_motion::fixed_frame{"Ground", rigid_motion::axes_settings{}})
            , panel("Frames", body, rigid_motion::baseline().frame, arranged(), std::string(), offered)
    {
        REQUIRE(body.initialize().has_value());
        panel.initialize();
        for(std::size_t index = 0; index < body.count(); ++index)
            REQUIRE_FALSE(body.pose(index).block<3, 3>(0, 0).isIdentity(1e-9));
    }

    moving_set(const moving_set &) = delete;

    threepp::Scene scene;
    rigid_motion::frame_stencil body;
    rigid_motion::frame_window panel;
};

inline rigid_motion::frame_window::controls every_panel()
{
    rigid_motion::frame_window::controls asked;
    asked.visibility = true;

    return asked;
}

inline void draw(rigid_motion::frame_window &panel)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);
    frames.draw([&panel] { panel.render(); });
}

// A control an object's own panel draws is drawn inside the identifier scope that panel pushes, so
// it is named by the object it belongs to as well as by its label.
inline void take_entry_on(rigid_motion::frame_window &panel, std::size_t scope, const char *label, std::size_t entry)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing over = [&panel] { panel.render(); };
    take_entry_on(frames, over, panel.display_name().c_str(), scope, label, entry);
}

// The panel's own values composed back into a pose, through the same bound operations the window
// assigns with, so a panel is compared against the pose the stencil actually carries.
inline transform composed(const rigid_motion::frame_window::placement &shown)
{
    const rigid_motion::frame_ops motions = rigid_motion::baseline().frame;
    const Eigen::Vector3d angles          = shown.euler_degrees.cast<double>() * radians_per_degree;

    return motions.transformation_matrix_from_rotation_position(motions.rotation_matrix_from_euler(angles, shown.order), shown.position.cast<double>());
}

}

#endif
