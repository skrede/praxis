#ifndef HPP_GUARD_PRAXIS_TESTS_RIGID_MOTION_FRAME_ROSTER_H
#define HPP_GUARD_PRAXIS_TESTS_RIGID_MOTION_FRAME_ROSTER_H

#include "panel_keys.h"
#include "imgui_frame.h"
#include "panel_labels.h"

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/frame_roster_window.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <imgui.h>

#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <functional>
#include <string_view>

namespace praxis::fixture {

inline std::vector<rigid_motion::stencil_object> rostered_objects()
{
    rigid_motion::object_body bare;
    bare.shape = rigid_motion::body_shape::none;

    return {rigid_motion::stencil_object{"Fixed", rigid_motion::axes_settings{}, bare}, rigid_motion::stencil_object{"One", rigid_motion::axes_settings{}, bare},
            rigid_motion::stencil_object{"Two", rigid_motion::axes_settings{}, bare}};
}

// The noun this fixture hands the roster to generate names from. No shipped scenario supplies it, so
// a generated name a case reads back can only have come from this choice.
inline constexpr const char *rostered_stem = "Marker";

// The composition's side of the one selection, staged over a plain value so that a case reads it
// directly rather than through a second window. The write brings the index inside the object set,
// which is what the selector a scenario composes does with what the roster hands it.
inline rigid_motion::frame_roster_window::selection_route route_over(std::size_t &standing, const rigid_motion::frame_stencil &body)
{
    const std::function<void(std::size_t)> write = [&standing, &body](std::size_t index)
    {
        const std::size_t carried = body.count();
        standing                  = carried == 0 ? 0 : (index < carried ? index : carried - 1);
    };

    return {[&standing] { return standing; }, write};
}

// One stencil and one roster over it, standing where a composition would have left the two, with the
// one selection they share held here. The bound operations are the reference ones, so nothing a case
// reads comes back from a binding that answers the same value whatever it is asked. An empty
// fixed-frame name is a composition that named none.
struct rostered
{
    explicit rostered(std::string anchored = std::string(), std::string stem = rostered_stem)
            : standing(0)
            , scene()
            , body(scene, rostered_objects(), rigid_motion::baseline().frame, rigid_motion::fixed_frame{std::move(anchored), rigid_motion::axes_settings{}})
            , roster("Roster", body, rigid_motion::axes_settings{}, rigid_motion::object_body{}, std::move(stem), route_over(standing, body))
    {
        REQUIRE(body.initialize().has_value());
    }

    rostered(const rostered &) = delete;

    std::size_t standing;
    threepp::Scene scene;
    rigid_motion::frame_stencil body;
    rigid_motion::frame_roster_window roster;
};

inline drawing over(rigid_motion::frame_roster_window &panel)
{
    return [&panel] { panel.render(); };
}

// Every vertex one drawing produced, so two panels are compared by what they put on screen rather
// than by how much of it there was.
inline std::size_t geometry_of(rigid_motion::frame_roster_window &panel)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);
    frames.draw(over(panel));

    REQUIRE(frames.has_draw_data());
    REQUIRE(frames.vertices() > 0);

    return frames.signature();
}

inline void press_below_top(rigid_motion::frame_roster_window &panel, std::size_t steps)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing draw = over(panel);
    stand_below_top(frames, draw, steps);
    tap(frames, draw, ImGuiKey_Space);
}

inline void press_on(rigid_motion::frame_roster_window &panel, const char *label)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing draw = over(panel);
    press_on(frames, draw, panel.display_name().c_str(), label);
}

// The interface library takes typed text from the character queue the platform layer feeds it, so a
// name is typed by activating the field, queueing the characters, committing them and then pressing
// the control that makes a frame.
inline void type_and_create(rostered &staged, const char *field, std::string_view named)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing draw       = over(staged.roster);
    const char *const window = staged.roster.display_name().c_str();

    stand_on(frames, draw, window, field);
    tap(frames, draw, ImGuiKey_Enter);
    for(const char letter : named)
        ImGui::GetIO().AddInputCharacter(static_cast<unsigned int>(letter));
    frames.draw_frame(draw);
    tap(frames, draw, ImGuiKey_Enter);
    press_on(frames, draw, window, "Create");
}

// Walks the panel from its first control onto each label in turn. The walk only goes downwards, so
// reaching them in the order given is what says the panel draws them in it.
inline void walk_onto_each(rigid_motion::frame_roster_window &panel, const std::vector<const char *> &labels)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    const drawing draw       = over(panel);
    const char *const window = panel.display_name().c_str();

    reach_top(frames, draw);
    for(const char *const label : labels)
        walk_onto(frames, draw, control_id(window, label), "'" + std::string(label) + "' below the control before it");
}

// One frame created under the name the roster generates for it, answered by the name it carries.
inline std::string created_by(rostered &staged)
{
    const expected<std::size_t, refusal> made = staged.roster.create({});
    REQUIRE(made.has_value());

    return std::string(staged.body.name_of(made.value()));
}

inline std::vector<std::string> names_of(const rigid_motion::frame_stencil &body)
{
    std::vector<std::string> named;
    for(std::size_t index = 0; index < body.count(); ++index)
        named.emplace_back(body.name_of(index));

    return named;
}

}

#endif
