#include "frame_selection.h"

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/frame_selector_window.h"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::rigid_motion;

namespace {

constexpr std::size_t opened_with = 3;

std::vector<std::string> every_name(const selecting &staged)
{
    return {std::string(staged.body.fixed_frame_name()), "One", "Two", "Three"};
}

}

// The frame the objects hang under has no placement to edit and takes no index, so a window whose
// whole purpose is to name one of the objects must not offer it.
TEST_CASE("the selector offers the stencil's objects and not the frame they hang under", "[rigid_motion][windows]")
{
    selecting staged;
    REQUIRE(staged.body.fixed_frame_name() == "Ground");

    const offering read = offered(staged.selector, every_name(staged));

    REQUIRE(read.carried == std::vector<std::size_t>{0, 1, 1, 1});
    REQUIRE(read.entries == opened_with);
}

TEST_CASE("taking an entry moves the selection in the stencil's own index space", "[rigid_motion][windows]")
{
    selecting staged;
    REQUIRE(staged.selector.selected_object() == 0u);

    for(std::size_t entry = 0; entry < opened_with; ++entry)
    {
        take_entry_on(staged.selector, "Object", entry);

        INFO("the entry taken was " << entry);
        REQUIRE(staged.selector.selected_object() == entry);
        REQUIRE(staged.body.name_of(entry) == selectable_objects()[entry].name);
    }
}

// Nothing is asked of the window between the two draws: the object set lives in the stencil and is
// read at every draw, so a removal made anywhere reaches the list and the selection on its own.
TEST_CASE("the selection follows the object set when the frame it stands on is taken away", "[rigid_motion][windows]")
{
    selecting staged;

    take_entry_on(staged.selector, "Object", opened_with - 1);

    REQUIRE(staged.selector.selected_object() == opened_with - 1);
    REQUIRE(staged.body.remove(opened_with - 1).has_value());

    draw_once(staged.selector);

    REQUIRE(staged.body.count() == opened_with - 1);
    REQUIRE(staged.selector.selected_object() == opened_with - 2);

    const offering read = offered(staged.selector, {"Three"});

    REQUIRE(read.entries == opened_with - 1);
    REQUIRE(read.carried == std::vector<std::size_t>{0});
}

TEST_CASE("an object added to the stencil directly is offered without the window being told", "[rigid_motion][windows]")
{
    selecting staged;

    staged.body.add(stencil_object{"Elbow", axes_settings{}, object_body{}});

    const offering read = offered(staged.selector, {"Elbow"});

    REQUIRE(read.entries == opened_with + 1);
    REQUIRE(read.carried == std::vector<std::size_t>{1});
}

// The tick is drawn over the one selection rather than inside a placement panel, so it reaches a
// frame no panel draws, and it reaches that frame alone.
TEST_CASE("the tick shows and hides the axes of the frame the selection names and no other", "[rigid_motion][windows]")
{
    selecting staged;
    REQUIRE(staged.selector.selected_object() == 0u);
    for(std::size_t index = 0; index < opened_with; ++index)
        REQUIRE(staged.body.axes_shown(index));

    press_on(staged.selector, "Axes");

    REQUIRE_FALSE(staged.body.axes_shown(0));
    REQUIRE(staged.body.axes_shown(1));
    REQUIRE(staged.body.axes_shown(2));

    take_entry_on(staged.selector, "Object", opened_with - 1);
    press_on(staged.selector, "Axes");

    REQUIRE(staged.selector.selected_object() == opened_with - 1);
    REQUIRE_FALSE(staged.body.axes_shown(0));
    REQUIRE(staged.body.axes_shown(1));
    REQUIRE_FALSE(staged.body.axes_shown(2));
}

TEST_CASE("the frame the selector stands on is drawn above the tick that shows its axes", "[rigid_motion][windows]")
{
    selecting staged;

    walk_onto_each(staged.selector, {"Object", "Axes"});
}

// The selection is written from outside between two draws, so it is brought inside the object set
// the stencil carries rather than inside the entries this window last drew.
TEST_CASE("a selection written past the end stands on the last object, and on none it stands at zero", "[rigid_motion][windows]")
{
    selecting staged;

    staged.selector.select_object(opened_with + 5);

    REQUIRE(staged.selector.selected_object() == opened_with - 1);

    staged.selector.select_object(1);

    REQUIRE(staged.selector.selected_object() == 1u);

    for(std::size_t left = opened_with; left > 0; --left)
        REQUIRE(staged.body.remove(left - 1).has_value());
    REQUIRE(staged.body.count() == 0u);

    staged.selector.select_object(opened_with);

    REQUIRE(staged.selector.selected_object() == 0u);
}
