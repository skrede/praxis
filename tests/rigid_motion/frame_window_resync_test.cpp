#include "frame_window_stage.h"

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/frame_window.h"

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <optional>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::rigid_motion;

namespace {

constexpr std::size_t opened_with = 4;

}

TEST_CASE("an object added to the stencil is drawn a panel on the window's next draw", "[rigid_motion][windows]")
{
    moving_set stage(every_panel());
    draw(stage.panel);

    REQUIRE(stage.panel.state().objects.size() == opened_with);

    stage.body.add(stencil_object{"Wrist", axes_settings{}, object_body{}});
    draw(stage.panel);

    REQUIRE(stage.panel.state().objects.size() == opened_with + 1);
    REQUIRE(stage.body.count() == opened_with + 1);
}

TEST_CASE("an object removed from the middle leaves every surviving panel reading that object's own pose", "[rigid_motion][windows]")
{
    moving_set stage(every_panel());
    draw(stage.panel);

    REQUIRE(stage.body.remove(1).has_value());
    draw(stage.panel);

    const frame_window::settings shown = stage.panel.state();

    REQUIRE(shown.objects.size() == stage.body.count());
    REQUIRE(shown.objects.size() == opened_with - 1);
    for(std::size_t index = 0; index < shown.objects.size(); ++index)
    {
        CAPTURE(index);
        REQUIRE(composed(shown.objects[index]).isApprox(stage.body.pose(index), 1e-5));
        REQUIRE(shown.objects[index].parent == stage.body.parent_of(index));
    }

    // The last object hung under the one before it, and both moved one place down.
    REQUIRE(shown.objects.back().parent == std::optional<std::size_t>(std::size_t{1}));
}

// The entry list every parent selection draws is rebuilt with the panel list: an object added at run
// time is choosable as a parent by a panel that was drawn before it existed.
TEST_CASE("an object added at run time is choosable as a parent by every other frame's panel", "[rigid_motion][windows]")
{
    moving_set stage(every_panel());
    draw(stage.panel);

    stage.body.add(stencil_object{"Wrist", axes_settings{}, object_body{}});
    draw(stage.panel);

    // Entry zero is the frame the objects hang under, so the object just added is the fifth below it.
    take_entry_on(stage.panel, 0, "Parent", opened_with + 1);

    REQUIRE(stage.body.parent_of(0) == std::optional<std::size_t>(opened_with));
    REQUIRE(stage.panel.state().objects.front().parent == std::optional<std::size_t>(opened_with));
}

TEST_CASE("a panel value stands while the object set keeps its size", "[rigid_motion][windows]")
{
    moving_set stage(every_panel());

    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);
    const drawing over = [&stage] { stage.panel.render(); };

    stand_on(frames, over, stage.panel.display_name().c_str(), 0, "Axis order");
    tap(frames, over, ImGuiKey_Space);
    tap(frames, over, ImGuiKey_RightArrow);

    const frame_window::placement edited = stage.panel.state().objects.front();

    // A pose written straight onto the stencil is what a resynchronization would read back; with the
    // count unmoved it is not asked for, and the panel keeps the value a person put there.
    stage.body.set_pose(0, transform::Identity());
    frames.draw(over);

    REQUIRE((stage.panel.state().objects.front().position - edited.position).isZero());
    REQUIRE((stage.panel.state().objects.front().euler_degrees - edited.euler_degrees).isZero());
    REQUIRE(stage.panel.state().objects.size() == opened_with);
}
