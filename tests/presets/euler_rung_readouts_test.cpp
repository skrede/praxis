#include "opened_rung.h"

#include "imgui_frame.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/frame_window.h"

#include "praxis/scene/imgui_window.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/core/Object3D.hpp>

#include <string>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::presets;
using namespace praxis::rigid_motion;

namespace {

// What one frame the readout draws puts on screen: the vertex count separates a rotation's grid
// from a transformation's, and the signature separates two readings of the same shape.
struct drawn
{
    explicit drawn(scene::imgui_window &window)
            : vertices(0)
            , signature(0u)
    {
        tests::imgui_frame frames;
        frames.assert_on_frame_faults(true);
        frames.draw([&window] { window.render(); });

        REQUIRE(frames.has_draw_data());
        REQUIRE(frames.vertices() > 0);

        vertices  = frames.vertices();
        signature = frames.signature();
    }

    int vertices;
    std::size_t signature;
};

// Turned as well as moved, so a rotation reading and a transformation reading both stand somewhere
// they did not stand before. The turn is composed through the reference implementation, because the
// inert one this repository also ships answers the identity to every angle.
transform displaced()
{
    transform put         = transform::Identity();
    put.block<3, 3>(0, 0) = baseline().frame.rotate_z(to_radians(30.0));
    put(0, 3)             = 1.25;

    return put;
}

// The mark node is raised for every stencil whether or not anything is marked, so a scene marking
// nothing is a node that stands there and is not visible rather than one that is absent. It composes
// its own placement inside the world-matrix update the renderer gives every node, so the scene is
// brought up to date first.
bool mark_drawn(opened_rung &rung)
{
    rung.body().render();
    rung.stage.updateMatrixWorld(true);

    threepp::Object3D *const drawn = rung.stage.getObjectByName("mark");
    REQUIRE(drawn != nullptr);

    return drawn->visible;
}

}

TEST_CASE("each rung composes a rotation readout and a transformation readout over the frame it reads", "[presets][windows]")
{
    const opened_rung beside(euler_rung::paired_frames);
    REQUIRE(beside.composed->windows.size() == 4u);

    scene::imgui_window &rotation       = *beside.composed->windows[2];
    scene::imgui_window &transformation = *beside.composed->windows[3];

    REQUIRE(rotation.display_name() == "Rotation");
    REQUIRE(transformation.display_name() == "Transformation");

    const drawn turned(rotation);
    const drawn placed(transformation);

    // A transformation carries the rotation and a row and a column beyond it, so it draws more.
    REQUIRE(placed.vertices > turned.vertices);

    beside.body().set_pose(1, displaced());

    REQUIRE(drawn(rotation).signature == turned.signature);
    REQUIRE(drawn(transformation).signature == placed.signature);

    beside.body().set_pose(0, displaced());

    REQUIRE(drawn(rotation).signature != turned.signature);
    REQUIRE(drawn(transformation).signature != placed.signature);
}

// Choosing a frame and reading one are two windows, and one choice drives both readouts: what says so
// is that they move together when the selection moves and stand still when a frame it does not name
// moves under them.
TEST_CASE("both of a rung's readouts read whichever frame its selector stands on", "[presets][windows]")
{
    const opened_rung beside(euler_rung::paired_frames);
    REQUIRE(beside.selector().selected_object() == 0u);

    scene::imgui_window &rotation       = *beside.composed->windows[2];
    scene::imgui_window &transformation = *beside.composed->windows[3];

    const std::size_t turned = drawn(rotation).signature;
    const std::size_t placed = drawn(transformation).signature;

    // The frame the selection does not stand on moving leaves both readouts where they were.
    beside.body().set_pose(1, displaced());

    REQUIRE(drawn(rotation).signature == turned);
    REQUIRE(drawn(transformation).signature == placed);

    take_entry_on(beside.selector(), "Object", 1);

    REQUIRE(beside.selector().selected_object() == 1u);
    REQUIRE_FALSE(beside.body().pose(1).isApprox(beside.body().pose(0)));
    REQUIRE(drawn(rotation).signature != turned);
    REQUIRE(drawn(transformation).signature != placed);
}

// The scene says which frame the rung's menus are about, on the same one choice the readouts follow:
// the mark stands on the frame the selector names, a frame it does not name moving leaves it where it
// was, and taking another entry moves it. The rung composed of one frame draws no mark, that frame
// standing apart from nothing.
TEST_CASE("a rung of one frame draws no mark, and a rung of two marks whichever frame its selector stands on", "[presets][windows]")
{
    opened_rung alone(euler_rung::single_frame);
    opened_rung beside(euler_rung::paired_frames);

    REQUIRE(alone.body().count() == 1u);
    REQUIRE_FALSE(mark_drawn(alone));

    REQUIRE(beside.body().count() == 2u);
    REQUIRE(beside.selector().selected_object() == 0u);

    const auto standing = marked_place(beside);

    beside.body().set_pose(1, displaced());

    REQUIRE(marked_place(beside) == standing);

    take_entry_on(beside.selector(), "Object", 1);

    REQUIRE(beside.selector().selected_object() == 1u);
    REQUIRE_FALSE(marked_place(beside) == standing);
}

TEST_CASE("a rung with no document present opens at the placements it carries in code", "[presets][windows]")
{
    const opened_rung alone(euler_rung::single_frame);
    const frame_window::settings opened = alone.panel().state();

    REQUIRE(opened.objects.size() == 1u);
    REQUIRE(opened.objects[0].euler_degrees.isZero());
    REQUIRE(opened.objects[0].position.z() > 0.f);

    const opened_rung beside(euler_rung::paired_frames);
    const frame_window::settings paired = beside.panel().state();

    REQUIRE(paired.objects.size() == 2u);
    REQUIRE(paired.objects[0].euler_degrees.isZero());
    REQUIRE_FALSE(paired.objects[0].position.isApprox(paired.objects[1].position));

    // Nothing hangs under anything until a panel puts it there.
    for(const frame_window::placement &one : paired.objects)
        REQUIRE_FALSE(one.parent.has_value());
}
