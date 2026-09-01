#include "frame_panel.h"

#include "captured_log.h"

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/frame_window.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/configuration.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <optional>
#include <filesystem>
#include <functional>
#include <string_view>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::rigid_motion;

namespace {

constexpr std::string_view at = "arrangement";

frame_window::controls panels_from(std::size_t first)
{
    frame_window::controls asked;
    asked.first_panelled_object = first;

    return asked;
}

// The choice of which frame a window is about is made outside it and read at every draw, which is
// how every composition that stands a window on one frame hands it that frame.
std::function<std::size_t()> standing(std::size_t index)
{
    return [index] { return index; };
}

// A document carrying one instance per object, every placement at the origin, so the window's own
// placements differ from it and there is something to offer.
config::document authored()
{
    const std::filesystem::path base  = std::filesystem::temp_directory_path();
    const std::filesystem::path where = base / "praxis-frame-window-controls.xml";

    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << "<probe><arrangement>";
    for(const stencil_object &object : three_objects())
        out << "<object name=\"" << object.name << "\" euler_order=\"ZYX\"><position x=\"0\" y=\"0\" z=\"0\"/><euler x=\"0\" y=\"0\" z=\"0\"/></object>";
    out << "</arrangement></probe>\n";
    out.close();

    config::declaration shape("probe");
    declare_arrangement(shape, at);

    const expected<config::document, config::error> read = config::load(shape, config::resolve(where, base));
    INFO((read ? std::string() : read.error().message));
    REQUIRE(read.has_value());

    return read.value();
}

}

TEST_CASE("a window composed with every control it draws today draws exactly what it draws today", "[rigid_motion][windows]")
{
    staged today;
    staged asked{frame_window::controls()};

    REQUIRE(geometry_of(asked.panel) == geometry_of(today.panel));
}

TEST_CASE("a window composed without the parent control draws no parent selection and none can be moved", "[rigid_motion][windows]")
{
    frame_window::controls asked;
    asked.parent = false;

    staged narrow(asked);
    staged wide{frame_window::controls()};

    REQUIRE(geometry_of(narrow.panel) != geometry_of(wide.panel));

    // The wider composition draws one parent selection per panelled object, and this one draws none.
    REQUIRE(items_offered(wide.panel) - items_offered(narrow.panel) == three_objects().size());

    drive(narrow.panel);

    REQUIRE_FALSE(narrow.body.parent_of(0).has_value());
    REQUIRE_FALSE(narrow.body.parent_of(1).has_value());
    REQUIRE(narrow.body.parent_of(2) == std::optional<std::size_t>(std::size_t{1}));
}

TEST_CASE("a window standing on a selection draws that frame's panel and follows the selection when it moves", "[rigid_motion][windows]")
{
    std::size_t chosen = 0;

    staged selecting(frame_window::controls(), [&chosen] { return chosen; });
    staged every{frame_window::controls()};

    REQUIRE(items_offered(selecting.panel) < items_offered(every.panel));
    REQUIRE(selecting.panel.selected_object() == 0u);

    const std::size_t first = geometry_of(selecting.panel);

    chosen = 1;

    REQUIRE(geometry_of(selecting.panel) != first);
    REQUIRE(selecting.panel.selected_object() == 1u);
}

TEST_CASE("a window composed with the visibility control hides the axes of the object whose panel it draws", "[rigid_motion][windows]")
{
    frame_window::controls asked;
    asked.visibility = true;

    staged showing(asked, standing(0));
    REQUIRE(showing.body.axes_shown(0));

    press_on(showing.panel, showing.panel.selected_object(), "Axes");

    REQUIRE_FALSE(showing.body.axes_shown(0));
    REQUIRE(showing.body.axes_shown(1));
    REQUIRE(showing.body.axes_shown(2));
}

TEST_CASE("a window composed without the visibility control leaves the axes where the composition set them", "[rigid_motion][windows]")
{
    staged narrow{frame_window::controls()};
    narrow.body.set_axes_shown(1, false);

    drive(narrow.panel);

    REQUIRE(narrow.body.axes_shown(0));
    REQUIRE_FALSE(narrow.body.axes_shown(1));
    REQUIRE(narrow.body.axes_shown(2));
}

TEST_CASE("the control set reaches nothing a window offers to write back", "[rigid_motion][windows]")
{
    const config::document carried = authored();

    threepp::Scene target;
    frame_stencil body(target, three_objects(), frame_ops{}, fixed_frame{"Ground", axes_settings{}});
    const frame_window few("Frame", body, frame_ops{}, resting(), std::string(at), panels_from(1));
    const frame_window many("Frame", body, frame_ops{}, resting(), std::string(at));

    const std::vector<config::edit> narrow = few.settings_edits(carried);
    const std::vector<config::edit> wide   = many.settings_edits(carried);

    REQUIRE_FALSE(wide.empty());
    REQUIRE(narrow.size() == wide.size());
    for(std::size_t which = 0; which < wide.size(); ++which)
    {
        INFO(wide[which].key);
        REQUIRE(narrow[which].key == wide[which].key);
        REQUIRE(narrow[which].value == wide[which].value);
    }
}

TEST_CASE("a window composed to leave the first object alone draws it no panel and still offers it as a parent", "[rigid_motion][windows]")
{
    staged narrow(panels_from(1));
    staged wide{frame_window::controls()};

    REQUIRE(items_offered(wide.panel) > items_offered(narrow.panel));

    const transform placed_before = narrow.body.pose(0);
    drive(narrow.panel);

    REQUIRE(narrow.body.pose(0).isApprox(placed_before));
    REQUIRE_FALSE(narrow.body.parent_of(0).has_value());
    REQUIRE(narrow.body.axes_shown(0));

    // Stepping over an object's panel does not take the object out of what a panel can hang under.
    staged selecting(panels_from(1), standing(1));
    take_entry_on(selecting.panel, selecting.panel.selected_object(), "Parent");
    REQUIRE(selecting.body.parent_of(1) == std::optional<std::size_t>(std::size_t{0}));
}

TEST_CASE("an object offered as its own parent is refused by name and the arrangement is left as it was", "[rigid_motion][windows]")
{
    constexpr std::size_t third = 2u;
    const std::optional<std::size_t> under_the_second(std::size_t{1});

    staged driven(frame_window::controls(), standing(third));
    staged untouched(frame_window::controls(), standing(third));

    REQUIRE(driven.body.parent_of(third) == under_the_second);

    // The parent list is the fixed frame followed by the objects in the stencil's order, so the last
    // entry names the object whose panel is being driven.
    const std::string reported = tests::reported_by(
            [&driven]
            {
                tests::imgui_frame frames;
                frames.assert_on_frame_faults(true);
                const drawing draw = over(driven.panel);
                take_entry_on(frames, draw, driven.panel.display_name().c_str(), third, "Parent", three_objects().size());
            });

    REQUIRE(reported.find("'Third' cannot be expressed in 'Third', so the arrangement is left as it was") != std::string::npos);
    REQUIRE(reported.find("[warning]") != std::string::npos);
    REQUIRE(driven.body.parent_of(third) == under_the_second);
    REQUIRE(driven.panel.state().objects[third].parent == under_the_second);
    REQUIRE(geometry_of(driven.panel) != geometry_of(untouched.panel));
}
