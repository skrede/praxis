#include "marked_stencil.h"

#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/frame_stencil.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/math/Vector3.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/core/Object3D.hpp>

#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::rigid_motion;

TEST_CASE("a stencil carrying one object draws no mark, there being nothing to tell it apart from")
{
    marked_scene built(one(described("Alone", true, body_shape::cube)));

    REQUIRE(built.body.count() == 1u);
    REQUIRE_FALSE(built.mark().visible);
}

// The rule reads the size of the object set and not the selection, so a selection standing squarely
// on the only object leaves it unmarked, and a second object arriving marks it without the selection
// having moved.
TEST_CASE("a lone object is unmarked while its selection names it, and is marked once a second object arrives")
{
    marked_scene built(one(described("Alone", true, body_shape::cube)));
    built.chosen = 0;

    REQUIRE_FALSE(built.mark().visible);
    REQUIRE(built.body.add(described("Second", true, body_shape::cube)) == 1u);

    threepp::Object3D &drawn = built.mark();

    REQUIRE(drawn.visible);
    REQUIRE(same_place(world_place(drawn), extent_of(node_named(built.stage, "Alone")).getCenter()));
}

TEST_CASE("a stencil carrying two objects stops marking when one of them is removed")
{
    marked_scene built(two_cubes());

    REQUIRE(built.mark().visible);
    REQUIRE(built.body.remove(1).has_value());
    REQUIRE(built.body.count() == 1u);
    REQUIRE_FALSE(built.mark().visible);
}

// Two objects stand here so that the missing selection is the only reason nothing is marked.
TEST_CASE("a stencil a composition hands no selection marks nothing")
{
    threepp::Scene target;
    frame_stencil body(target, two_cubes(), frame_ops{});

    REQUIRE(body.initialize().has_value());
    REQUIRE(body.count() == 2u);
    REQUIRE_FALSE(mark_in(target, body).visible);
}

// Two objects stand here so that the selection reaching past them is the only reason nothing is
// marked.
TEST_CASE("a selection standing past the end of the object set marks nothing")
{
    marked_scene built(two_cubes());

    REQUIRE(built.mark().visible);

    built.chosen = built.body.count();

    REQUIRE_FALSE(built.mark().visible);
}

TEST_CASE("no index names the fixed frame, so a stencil emptied of objects marks nothing while it is still drawn")
{
    marked_scene built(one(described("Frame", true, body_shape::cube)), fixed_frame{"Reference", axes_settings{}});

    REQUIRE(built.body.count() == 1u);
    REQUIRE(built.body.remove(0).has_value());
    REQUIRE(built.body.count() == 0u);
    REQUIRE(built.stage.getObjectByName("Reference") != nullptr);
    REQUIRE_FALSE(built.mark().visible);
}
