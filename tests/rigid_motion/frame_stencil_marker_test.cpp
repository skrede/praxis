#include "marked_stencil.h"

#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/frame_stencil.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <threepp/math/Color.hpp>
#include <threepp/math/Vector3.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/core/Object3D.hpp>

#include <threepp/materials/LineBasicMaterial.hpp>

#include <memory>
#include <cstddef>
#include <utility>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::rigid_motion;

namespace {

// A mark is line geometry, so its tone hangs on a line material rather than on the lit material a
// body carries.
threepp::Color mark_tone(threepp::Object3D &drawn)
{
    auto *const lines = drawn.materialAs<threepp::LineBasicMaterial>();
    REQUIRE(lines != nullptr);

    return lines->color;
}

const std::pair<threepp::Color, const char *> scene_tones[]{
        {threepp::Color::aliceblue, "the clear color"}, {threepp::Color::yellowgreen, "the grid"}, {threepp::Color::steelblue, "a cube body"},  {threepp::Color::red, "the x axis"},
        {threepp::Color::green, "the y axis"},          {threepp::Color::blue, "the z axis"},      {threepp::Color::white, "the origin marker"}};

}

TEST_CASE("the mark is drawn in a tone the scene draws nowhere else")
{
    marked_scene built(two_cubes());
    const threepp::Color marked = mark_tone(built.mark());

    for(const auto &[drawn, called] : scene_tones)
    {
        INFO("the mark stands apart from " << called);
        REQUIRE_FALSE(marked.equals(drawn));
    }
}

TEST_CASE("the mark stands where the frame the selection names stands, and moves when that selection moves")
{
    marked_scene built(two_cubes());
    built.chosen = 1;
    built.body.set_pose(0, placed_at(-0.6, 0.0, 0.2));
    built.body.set_pose(1, placed_at(0.6, 0.0, 0.4));

    threepp::Object3D &drawn = built.mark();

    REQUIRE(drawn.visible);
    REQUIRE(same_place(world_place(drawn), extent_of(node_named(built.stage, "Second")).getCenter()));
    REQUIRE_FALSE(same_place(world_place(drawn), extent_of(node_named(built.stage, "First")).getCenter()));

    built.chosen = 0;

    REQUIRE(same_place(world_place(built.mark()), extent_of(node_named(built.stage, "First")).getCenter()));
}

TEST_CASE("an object carrying no body at all is marked")
{
    marked_scene built(accompanied(described("Bare", true, body_shape::none)));

    REQUIRE(node_named(built.stage, "Bare").getObjectByName("body") == nullptr);

    threepp::Object3D &drawn = built.mark();

    REQUIRE(drawn.visible);
    REQUIRE(drawn.scale.x > 0.f);
    REQUIRE(drawn.scale.y > 0.f);
    REQUIRE(drawn.scale.z > 0.f);
}

TEST_CASE("the material a marked object draws with is the one it was built with")
{
    marked_scene built(two_cubes());
    const threepp::Color made = drawn_tone(built.stage, "First");

    REQUIRE(made.equals(threepp::Color::steelblue));

    for(const std::size_t standing_on : {std::size_t{0}, std::size_t{1}, std::size_t{0}})
    {
        built.chosen = standing_on;
        built.mark();

        INFO("the object's own material was written while the selection stood on " << standing_on);
        REQUIRE(drawn_tone(built.stage, "First").equals(made));
    }
}

TEST_CASE("a mesh body a caller loaded is the object it loaded after the mark")
{
    stencil_object carried                          = described("Loaded", true, body_shape::mesh);
    carried.body.mesh                               = loaded_body();
    const std::shared_ptr<threepp::Object3D> handed = carried.body.mesh;
    const std::size_t hung                          = handed->children.size();

    marked_scene built(accompanied(std::move(carried)));

    REQUIRE(built.mark().visible);
    REQUIRE(node_named(built.stage, "Loaded").getObjectByName("body")->children.front() == handed.get());
    REQUIRE(handed->children.size() == hung);
}

TEST_CASE("an object whose axes are hidden is marked at the extent it stands at")
{
    marked_scene built(accompanied(described("Seen", true, body_shape::none)));

    const threepp::Vector3 place = world_place(built.mark());
    const threepp::Vector3 reach = built.mark().scale;

    built.body.set_axes_shown(0, false);

    threepp::Object3D &blind = built.mark();

    REQUIRE(blind.visible);
    REQUIRE(same_place(world_place(blind), place));
    REQUIRE(same_place(blind.scale, reach));
}

TEST_CASE("the mark stands off the extent of what it marks by the same margin in every direction")
{
    marked_scene built(accompanied(described("Frame", true, body_shape::cube)));
    built.body.set_pose(0, placed_at(0.3, -0.2, 0.4));

    const threepp::Vector3 reach = built.mark().scale;
    const threepp::Vector3 tight = extent_of(node_named(built.stage, "Frame")).getSize();
    const float along_x          = reach.x - tight.x / 2.f;

    REQUIRE(along_x > 0.f);
    REQUIRE(reach.y - tight.y / 2.f == Catch::Approx(along_x));
    REQUIRE(reach.z - tight.z / 2.f == Catch::Approx(along_x));
}

TEST_CASE("moving the object the mark stands on moves the mark with it")
{
    marked_scene built(accompanied(described("Frame", true, body_shape::cube)));
    built.body.set_pose(0, placed_at(0.2, 0.0, 0.3));

    const threepp::Vector3 first = world_place(built.mark());

    built.body.set_pose(0, placed_at(0.5, 0.0, 0.3));

    REQUIRE(same_place(world_place(built.mark()), threepp::Vector3{first.x + 0.3f, first.y, first.z}));
}
