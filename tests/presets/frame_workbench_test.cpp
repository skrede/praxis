#include "opened_workbench.h"

#include "composed_panels.h"

#include "praxis/presets/frame_workbench.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::presets;
using namespace praxis::rigid_motion;

TEST_CASE("the workbench loads, renders and unloads leaving the scene as it found it", "[presets][lifetime]")
{
    threepp::Scene stage;
    const std::size_t before = descendants(stage);

    {
        const std::shared_ptr<scene::preset> composed = frame_workbench_preset(unwired(stage), baseline());

        REQUIRE(composed != nullptr);
        REQUIRE(composed->initialize().has_value());
        REQUIRE(descendants(stage) > before);

        for(const std::shared_ptr<scene::imgui_window> &window : composed->windows)
        {
            window->initialize();
            geometry_of(*window);
        }
        each_window_opens_one_panel(composed);
        composed->stencil->tear_down();
    }

    REQUIRE(descendants(stage) == before);
}

TEST_CASE("a frame created from the roster is drawn a panel and is choosable as a parent", "[presets][windows]")
{
    const opened_workbench built;

    press_on(built.roster(), "Create");

    REQUIRE(built.body().count() == opened_with + 1);
    REQUIRE(built.selector().selected_object() == opened_with);

    geometry_of(built.panel());

    REQUIRE(built.panel().state().objects.size() == opened_with + 1);
    REQUIRE(built.panel().selected_object() == opened_with);

    take_entry_on(built.selector(), "Object", 0);
    geometry_of(built.panel());

    REQUIRE(built.panel().selected_object() == 0u);

    // Entry zero is the fixed frame the objects hang under, so the frame just created is the third
    // entry below it.
    take_entry_on(built.panel(), 0, "Parent", opened_with + 1);

    REQUIRE(built.body().parent_of(0) == std::optional<std::size_t>(opened_with));
}

TEST_CASE("a frame another frame is expressed in is not removed, and the roster shows the refusal", "[presets][windows]")
{
    const opened_workbench built;
    REQUIRE(built.body().set_parent(0, 1).has_value());

    press_at(built.roster(), row_step(1));

    REQUIRE(built.selector().selected_object() == 1u);

    const std::size_t drawn = geometry_of(built.roster());

    press_on(built.roster(), "Remove");

    REQUIRE(built.body().count() == opened_with);
    REQUIRE(built.body().parent_of(0) == std::optional<std::size_t>(std::size_t{1}));
    REQUIRE(geometry_of(built.roster()) != drawn);
}

TEST_CASE("a parent already descending from the frame it would be given to is refused and shown", "[presets][windows]")
{
    const opened_workbench built;
    REQUIRE(built.body().set_parent(1, 0).has_value());

    // The panel opens on the first object, which is the one the other now descends from.
    REQUIRE(built.panel().selected_object() == 0u);

    const std::size_t drawn = geometry_of(built.panel());

    take_entry_on(built.panel(), built.panel().selected_object(), "Parent", opened_with);

    REQUIRE_FALSE(built.body().parent_of(0).has_value());
    REQUIRE(geometry_of(built.panel()) != drawn);
}

TEST_CASE("removing an unrelated frame leaves a parent relation naming the frame it named", "[presets][windows]")
{
    const opened_workbench built;

    press_on(built.roster(), "Create");
    geometry_of(built.panel());

    REQUIRE(built.body().count() == opened_with + 1);
    REQUIRE(built.body().set_parent(opened_with, 0).has_value());
    REQUIRE(built.body().name_of(0) == "Frame");

    REQUIRE(built.body().remove(1).has_value());
    geometry_of(built.panel());

    REQUIRE(built.body().count() == opened_with);
    REQUIRE_FALSE(built.body().name_of(1).empty());
    REQUIRE(built.body().parent_of(1) == std::optional<std::size_t>(std::size_t{0}));
    REQUIRE(built.body().name_of(*built.body().parent_of(1)) == "Frame");
    REQUIRE(built.panel().state().objects[1].parent == std::optional<std::size_t>(std::size_t{0}));
}

TEST_CASE("the workbench composes five windows under the titles it names them by", "[presets][windows]")
{
    const opened_workbench built;

    REQUIRE(composed_windows(built.composed) == std::vector<std::string>{"Frame selector", "Frame parameters", "Frame roster", "Rotation", "Transformation"});
}

// Choosing a frame from either window, editing one, reading one, seeing which one is meant and
// showing its axes are five readers of one choice: what says so is that all five move together
// whether the choice is made in the roster or in the selector, and that a frame the choice does not
// name moving under them moves none of them.
TEST_CASE("one selection moves the parameters panel, both readouts, the mark in the scene and the axis tick", "[presets][windows]")
{
    opened_workbench built;
    REQUIRE(built.selector().selected_object() == 0u);

    built.body().set_axes_shown(0, false);

    const read_by_selection standing(built);
    built.body().set_pose(1, turned_and_moved());

    REQUIRE(read_by_selection(built) == standing);
    REQUIRE_FALSE(built.body().pose(1).isApprox(built.body().pose(0)));

    press_at(built.roster(), row_step(1));

    const read_by_selection pressed(built);
    REQUIRE(built.selector().selected_object() == 1u);
    REQUIRE(pressed.axes != standing.axes);
    REQUIRE(pressed.panel != standing.panel);
    REQUIRE(pressed.rotation != standing.rotation);
    REQUIRE(pressed.transformation != standing.transformation);
    REQUIRE(pressed.mark != standing.mark);

    take_entry_on(built.selector(), "Object", 0);

    const read_by_selection taken(built);
    REQUIRE(built.selector().selected_object() == 0u);
    REQUIRE(taken.axes != pressed.axes);
    REQUIRE(taken.panel != pressed.panel);
    REQUIRE(taken.rotation != pressed.rotation);
    REQUIRE(taken.transformation != pressed.transformation);
    REQUIRE(taken.mark != pressed.mark);
}

TEST_CASE("the fixed frame is drawn and choosable as a parent, and is in no list of objects", "[presets][windows]")
{
    opened_workbench built;
    const std::string named(built.body().fixed_frame_name());

    REQUIRE_FALSE(named.empty());
    REQUIRE(built.stage.getObjectByName(named) != nullptr);

    for(std::size_t index = 0; index < built.body().count(); ++index)
        REQUIRE(built.body().name_of(index) != named);
    REQUIRE(built.body().count() == opened_with);

    first_control_is(built.roster(), 0, "Frame");

    // Entry zero is the fixed frame and every later entry is one object, so the last entry names the
    // last object and entry zero names no object at all.
    take_entry_on(built.panel(), built.panel().selected_object(), "Parent", opened_with);

    REQUIRE(built.body().parent_of(0) == std::optional<std::size_t>(opened_with - 1));

    take_entry_on(built.panel(), built.panel().selected_object(), "Parent", 0);

    REQUIRE_FALSE(built.body().parent_of(0).has_value());
}
