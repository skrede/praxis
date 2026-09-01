#include "praxis/presets/frame_workbench.h"

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/frame_window.h"
#include "praxis/rigid_motion/configuration.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/matrix_readout.h"
#include "praxis/rigid_motion/frame_roster_window.h"
#include "praxis/rigid_motion/frame_selector_window.h"

#include "praxis/scene/imgui_window.h"
#include "praxis/scene/labeled_value_window.h"

#include <Eigen/Core>

#include <spdlog/spdlog.h>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <functional>

namespace praxis::presets {

namespace {

constexpr double body_edge     = 0.30;
constexpr float resting_height = 0.5f;
constexpr float standing_apart = 0.6f;

// The objects the workbench opens with, in the order the stencil draws them, which is the order a
// name in the document is turned back into an index against.
const std::vector<std::string> &opening_objects()
{
    static const std::vector<std::string> named{"Frame", "Second"};

    return named;
}

rigid_motion::object_body arriving_body()
{
    return rigid_motion::object_body{rigid_motion::body_shape::cube, body_edge, nullptr};
}

std::vector<rigid_motion::stencil_object> workbench_stencil()
{
    std::vector<rigid_motion::stencil_object> drawn;
    for(const std::string &named : opening_objects())
        drawn.push_back(rigid_motion::stencil_object{named, rigid_motion::axes_settings{}, arriving_body()});

    return drawn;
}

// Axis visibility is left off here because the selector carries it over the one selection, so one
// control reaches every frame rather than one control per panel.
rigid_motion::frame_window::controls workbench_controls()
{
    rigid_motion::frame_window::controls offered;
    offered.parent     = true;
    offered.visibility = false;

    return offered;
}

rigid_motion::frame_window::placement standing(float along)
{
    return rigid_motion::frame_window::placement{axis_order::zyx, Eigen::Vector3f{along, 0.f, resting_height}, Eigen::Vector3f{0.f, 0.f, 0.f}, std::nullopt};
}

// No placement names a parent, so every parent relation an arrangement stands in is one the panel
// put there.
rigid_motion::frame_window::settings resting_arrangement()
{
    return rigid_motion::frame_window::settings{{standing(standing_apart), standing(-standing_apart)}};
}

rigid_motion::frame_window::settings workbench_opening(const arrangement_source &arrangement)
{
    const rigid_motion::frame_window::settings resting = resting_arrangement();
    if(!arrangement.values)
        return resting;

    const expected<rigid_motion::frame_window::settings, config::error> read = rigid_motion::read_arrangement(*arrangement.values, arrangement.at, opening_objects(), resting);
    if(read)
        return read.value();

    spdlog::warn("praxis: {}, so the frame arrangement opens at the placements the scenario supplies", read.error().message);

    return resting;
}

// The readout is held by the two callables its window draws through, so it lives exactly as long as
// the window does; what it asks which frame to read is held the same way.
std::shared_ptr<scene::imgui_window> matrix_window(std::string name, const rigid_motion::frame_stencil &read, rigid_motion::matrix_form drawn, std::function<std::size_t()> selecting)
{
    const auto held = std::make_shared<rigid_motion::matrix_readout>(read, drawn, std::move(selecting));

    return std::make_shared<scene::labeled_value_window>(std::move(name), [held] { held->render_controls(); }, [held] { return held->reading(); });
}

}

std::shared_ptr<scene::preset> frame_workbench_preset(const scene::preset_site &site, const rigid_motion::capabilities &motions, arrangement_source arrangement)
{
    const rigid_motion::frame_window::settings opening = workbench_opening(arrangement);

    auto body = std::make_shared<rigid_motion::frame_stencil>(site.scene, workbench_stencil(), motions.frame, rigid_motion::fixed_frame{"Space", rigid_motion::axes_settings{}});
    const auto selector                       = std::make_shared<rigid_motion::frame_selector_window>("Frame selector", *body);
    const std::function<std::size_t()> chosen = [selector] { return selector->selected_object(); };
    const rigid_motion::frame_roster_window::selection_route standing{chosen, [selector](std::size_t index) { selector->select_object(index); }};
    body->follow_selection(chosen);
    const std::vector<std::shared_ptr<scene::imgui_window>> windows{
            selector,
            std::make_shared<rigid_motion::frame_window>("Frame parameters", *body, motions.frame, opening, std::move(arrangement.at), workbench_controls(), chosen),
            std::make_shared<rigid_motion::frame_roster_window>("Frame roster", *body, rigid_motion::axes_settings{}, arriving_body(), "Frame", standing),
            matrix_window("Rotation", *body, rigid_motion::matrix_form::rotation, chosen),
            matrix_window("Transformation", *body, rigid_motion::matrix_form::transformation, chosen),
    };

    return std::make_shared<scene::preset>(body, windows, site.add_window, site.remove_window);
}

}
