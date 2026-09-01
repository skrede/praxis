#include "opened_rung.h"

#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/frame_window.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/core/Object3D.hpp>

#include <threepp/math/Vector3.hpp>

#include <string>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::presets;
using namespace praxis::rigid_motion;

namespace {

constexpr float place_tolerance = 1.0e-6f;

// The node the fixed frame hangs in the composed scene, found by the name the composition gave it.
threepp::Object3D &anchor_of(opened_rung &rung)
{
    threepp::Object3D *const node = rung.stage.getObjectByName(std::string(rung.body().fixed_frame_name()));
    REQUIRE(node != nullptr);

    return *node;
}

bool at_origin(threepp::Object3D &node)
{
    threepp::Vector3 put;
    node.getWorldPosition(put);

    return put.length() < place_tolerance;
}

}

TEST_CASE("the first rung offers no parent choice for the frame it stands on, and the second offers one", "[presets][windows]")
{
    const opened_rung alone(euler_rung::single_frame);
    frame_window widened("Frame parameters", alone.body(), baseline().frame, alone.panel().state(), std::string(), asking(true, 0), about(0));

    // One parent selection for the one frame the panel stands on.
    REQUIRE(items_of(widened) - items_of(alone.panel()) == 1u);

    const opened_rung beside(euler_rung::paired_frames);
    frame_window narrowed("Frame parameters", beside.body(), baseline().frame, beside.panel().state(), std::string(), asking(false, 0), about(0));

    REQUIRE(items_of(beside.panel()) - items_of(narrowed) == 1u);
}

// A rung composes a selector and no roster, and the tick sits in the selector, so a scenario carrying
// a fixed set of frames still reaches it. The panel keeps the capability and this composition
// declines it: a panel asking for the tick over the same stencil draws one item more.
TEST_CASE("a rung reaches the axis tick through its selector and its parameters panel draws none", "[presets][windows]")
{
    opened_rung alone(euler_rung::single_frame);
    frame_window::controls ticked = asking(false, 0);
    ticked.visibility             = true;

    frame_window widened("Frame parameters", alone.body(), baseline().frame, alone.panel().state(), std::string(), ticked, about(0));

    REQUIRE(items_of(widened) - items_of(alone.panel()) == 1u);
    REQUIRE(alone.selector().selected_object() == 0u);
    REQUIRE(alone.body().axes_shown(0));

    press_on(alone.selector(), "Axes");

    REQUIRE_FALSE(alone.body().axes_shown(0));
}

TEST_CASE("neither rung draws a panel for the fixed frame, and nothing either panel offers moves it", "[presets][windows]")
{
    opened_rung alone(euler_rung::single_frame);
    opened_rung beside(euler_rung::paired_frames);

    // Nothing either stencil counts carries the fixed frame's name, so there is no index a panel
    // could draw it under.
    REQUIRE(alone.body().count() == 1u);
    REQUIRE(beside.body().count() == 2u);
    REQUIRE_FALSE(alone.body().fixed_frame_name().empty());
    REQUIRE(alone.body().name_of(0) != alone.body().fixed_frame_name());
    for(std::size_t index = 0; index < beside.body().count(); ++index)
        REQUIRE(beside.body().name_of(index) != beside.body().fixed_frame_name());

    REQUIRE(at_origin(anchor_of(alone)));
    REQUIRE(at_origin(anchor_of(beside)));

    drive_panel(alone.panel());
    drive_panel(beside.panel());

    REQUIRE(alone.body().count() == 1u);
    REQUIRE(beside.body().count() == 2u);
    REQUIRE(at_origin(anchor_of(alone)));
    REQUIRE(at_origin(anchor_of(beside)));
}

TEST_CASE("the fixed frame opens hidden on the first rung and drawn on the second", "[presets][windows]")
{
    opened_rung alone(euler_rung::single_frame);
    opened_rung beside(euler_rung::paired_frames);

    REQUIRE_FALSE(anchor_of(alone).visible);
    REQUIRE(anchor_of(beside).visible);

    for(std::size_t index = 0; index < alone.body().count(); ++index)
        REQUIRE(alone.body().axes_shown(index));
    for(std::size_t index = 0; index < beside.body().count(); ++index)
        REQUIRE(beside.body().axes_shown(index));
}
