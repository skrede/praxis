#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/frame_stencil.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/objects/Group.hpp>
#include <threepp/scenes/Scene.hpp>
#include <threepp/core/Object3D.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>

using namespace praxis;
using namespace praxis::rigid_motion;

namespace {

constexpr double chain_tolerance = 1.0e-9;

std::size_t descendants(threepp::Object3D &root)
{
    std::size_t counted = 0;
    root.traverse([&counted](threepp::Object3D &) { ++counted; });

    return counted;
}

stencil_object described(std::string name, body_shape shape)
{
    stencil_object object;
    object.name       = std::move(name);
    object.axes.shown = true;
    object.body.shape = shape;
    if(shape == body_shape::mesh)
        object.body.mesh = threepp::Group::create();

    return object;
}

// Every placement carries a rotation of its own, so a survivor whose rotation was dropped or written
// over reads differently from the survivor that was left alone.
transform turned(double radians, const Eigen::Vector3d &at)
{
    transform tf         = transform::Identity();
    tf.block<3, 3>(0, 0) = Eigen::AngleAxisd(radians, Eigen::Vector3d{1.0, -2.0, 3.0}.normalized()).toRotationMatrix();
    tf.block<3, 1>(0, 3) = at;

    return tf;
}

struct reading
{
    std::string name;
    transform placement;
    transform world;
    transform expressed_in;
    bool parented;
    bool shown;
};

reading read_one(const frame_stencil &body, std::size_t index)
{
    const std::optional<std::size_t> above = body.parent_of(index);

    return reading{
            std::string(body.name_of(index)), body.pose(index), body.world_pose(index), above ? body.pose(*above) : transform::Identity(), above.has_value(), body.axes_shown(index)};
}

bool reads_the_same(const reading &put, const reading &was)
{
    return put.name == was.name && put.placement.isApprox(was.placement, chain_tolerance) && put.world.isApprox(was.world, chain_tolerance) &&
            put.expressed_in.isApprox(was.expressed_in, chain_tolerance) && put.parented == was.parented && put.shown == was.shown;
}

std::vector<reading> read_every(const frame_stencil &body)
{
    std::vector<reading> taken;
    for(std::size_t index = 0; index < body.count(); ++index)
        taken.push_back(read_one(body, index));

    return taken;
}

std::vector<stencil_object> three_objects()
{
    std::vector<stencil_object> objects;
    objects.push_back(described("First", body_shape::cube));
    objects.push_back(described("Second", body_shape::cube));
    objects.push_back(described("Third", body_shape::cube));

    return objects;
}

void place_three(frame_stencil &body)
{
    body.set_pose(0, turned(0.4, Eigen::Vector3d{0.3, -0.2, 0.9}));
    body.set_pose(1, turned(-0.9, Eigen::Vector3d{1.1, 0.5, -0.4}));
    body.set_pose(2, turned(1.7, Eigen::Vector3d{-0.6, 0.8, 0.2}));
}

}

TEST_CASE("an object joins the scene with the drawables it carries and answers its index")
{
    threepp::Scene target;
    frame_stencil body(target, three_objects(), baseline().frame);
    REQUIRE(body.initialize().has_value());
    place_three(body);

    const std::size_t before = descendants(target);
    const std::size_t joined = body.add(described("Fourth", body_shape::cube));

    REQUIRE(joined == 3);
    REQUIRE(body.count() == 4);
    REQUIRE(body.name_of(joined) == "Fourth");
    REQUIRE(body.pose(joined).isApprox(transform::Identity()));

    threepp::Object3D *const node = target.getObjectByName("Fourth");
    REQUIRE(node != nullptr);
    REQUIRE(descendants(target) == before + descendants(*node));

    // The tree grew with the object set: a parent naming the new frame is one the tree can serve.
    REQUIRE(body.set_parent(joined, std::size_t{0}).has_value());
    REQUIRE(body.world_pose(joined).isApprox(body.pose(0), chain_tolerance));
}

TEST_CASE("adding an object moves no object already placed")
{
    threepp::Scene target;
    frame_stencil body(target, three_objects(), baseline().frame);
    REQUIRE(body.initialize().has_value());
    place_three(body);
    REQUIRE(body.set_parent(2, std::size_t{1}).has_value());

    const std::vector<reading> before = read_every(body);
    body.add(described("Fourth", body_shape::none));
    const std::vector<reading> after = read_every(body);

    for(std::size_t index = 0; index < before.size(); ++index)
        REQUIRE(reads_the_same(after[index], before[index]));
}

TEST_CASE("an object removed from the middle leaves every survivor reading what it read")
{
    threepp::Scene target;
    frame_stencil body(target, three_objects(), baseline().frame);
    REQUIRE(body.initialize().has_value());
    place_three(body);
    body.add(described("Fourth", body_shape::cube));
    body.set_pose(3, turned(0.55, Eigen::Vector3d{0.2, 1.3, -0.7}));
    REQUIRE(body.set_parent(3, std::size_t{2}).has_value());
    body.set_axes_shown(3, false);

    const std::vector<reading> before = read_every(body);
    REQUIRE(body.remove(1).has_value());
    const std::vector<reading> after = read_every(body);

    REQUIRE(body.count() == 3);
    REQUIRE(target.getObjectByName("Second") == nullptr);
    // The object before the removed one stayed where it was; everything after it moved down one.
    REQUIRE(reads_the_same(after[0], before[0]));
    for(std::size_t index = 1; index < after.size(); ++index)
        REQUIRE(reads_the_same(after[index], before[index + 1]));

    REQUIRE(body.name_of(1) == "Third");
    REQUIRE(body.name_of(2) == "Fourth");
    REQUIRE(body.parent_of(2) == std::optional<std::size_t>{1});
    REQUIRE_FALSE(body.axes_shown(2));
}

TEST_CASE("the stencil refuses a removal exactly where the tree does and touches nothing")
{
    threepp::Scene target;
    frame_stencil body(target, three_objects(), baseline().frame);
    REQUIRE(body.initialize().has_value());
    place_three(body);
    REQUIRE(body.set_parent(2, std::size_t{1}).has_value());

    const std::size_t hung            = descendants(target);
    const std::vector<reading> before = read_every(body);

    SECTION("a frame another object's placement is expressed in")
    {
        const expected<void, refusal> refused = body.remove(1);
        REQUIRE_FALSE(refused.has_value());
        REQUIRE(refused.error() == refusal::no_solution);
        REQUIRE(target.getObjectByName("Second") != nullptr);
    }

    SECTION("an index past the end")
    {
        const expected<void, refusal> refused = body.remove(9);
        REQUIRE_FALSE(refused.has_value());
        REQUIRE(refused.error() == refusal::unsupported_input);
    }

    const std::vector<reading> after = read_every(body);
    REQUIRE(body.count() == before.size());
    REQUIRE(descendants(target) == hung);
    for(std::size_t index = 0; index < after.size(); ++index)
        REQUIRE(reads_the_same(after[index], before[index]));
    REQUIRE(body.parent_of(2) == std::optional<std::size_t>{1});
}

TEST_CASE("an object that joins and leaves takes its nodes and its shares with it")
{
    threepp::Scene target;
    frame_stencil body(target, three_objects(), baseline().frame);
    REQUIRE(body.initialize().has_value());
    place_three(body);

    const std::size_t hung = descendants(target);

    stencil_object leaving                      = described("Gone", body_shape::mesh);
    const std::weak_ptr<threepp::Object3D> held = leaving.body.mesh;
    const std::size_t joined                    = body.add(std::move(leaving));

    REQUIRE_FALSE(held.expired());
    REQUIRE(descendants(target) > hung);

    REQUIRE(body.remove(joined).has_value());

    REQUIRE(body.count() == 3);
    REQUIRE(descendants(target) == hung);
    REQUIRE(target.getObjectByName("Gone") == nullptr);
    REQUIRE(held.expired());
}

TEST_CASE("a stencil that gains and loses an object mid-life leaves the scene as it found it")
{
    threepp::Scene target;
    const std::size_t empty = descendants(target);

    frame_stencil body(target, three_objects(), baseline().frame);
    REQUIRE(body.initialize().has_value());
    place_three(body);
    body.render();

    const std::size_t hung = descendants(target);
    body.add(described("Fourth", body_shape::cube));
    body.set_pose(3, turned(0.55, Eigen::Vector3d{0.2, 1.3, -0.7}));
    body.render();

    REQUIRE(target.getObjectByName("Fourth") != nullptr);
    REQUIRE(body.remove(3).has_value());
    body.render();

    REQUIRE(descendants(target) == hung);
    REQUIRE(body.world_pose(2).isApprox(turned(1.7, Eigen::Vector3d{-0.6, 0.8, 0.2}), chain_tolerance));

    body.tear_down();
    REQUIRE(descendants(target) == empty);
    REQUIRE(target.getObjectByName("Third") == nullptr);
}

TEST_CASE("a scrambled sequence of adds and removes ends with the objects it should have")
{
    threepp::Scene target;
    frame_stencil body(target, three_objects(), baseline().frame);
    REQUIRE(body.initialize().has_value());
    place_three(body);

    const transform third_at  = turned(1.7, Eigen::Vector3d{-0.6, 0.8, 0.2});
    const transform fourth_at = turned(0.55, Eigen::Vector3d{0.2, 1.3, -0.7});

    body.add(described("Fourth", body_shape::cube));
    body.set_pose(3, fourth_at);
    body.add(described("Fifth", body_shape::none));
    REQUIRE(body.set_parent(4, std::size_t{3}).has_value());
    REQUIRE(body.set_parent(3, std::size_t{2}).has_value());
    REQUIRE(body.remove(0).has_value());
    REQUIRE(body.remove(0).has_value());
    body.add(described("Sixth", body_shape::cube));
    REQUIRE(body.set_parent(3, std::size_t{2}).has_value());

    const expected<void, refusal> refused = body.remove(2);
    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == refusal::no_solution);

    REQUIRE(body.set_parent(3, std::nullopt).has_value());
    REQUIRE(body.remove(2).has_value());

    REQUIRE(body.count() == 3);
    REQUIRE(body.name_of(0) == "Third");
    REQUIRE(body.name_of(1) == "Fourth");
    REQUIRE(body.name_of(2) == "Sixth");
    REQUIRE(body.parent_of(0) == std::nullopt);
    REQUIRE(body.parent_of(1) == std::optional<std::size_t>{0});
    REQUIRE(body.parent_of(2) == std::nullopt);
    REQUIRE(body.pose(0).isApprox(third_at));
    REQUIRE(body.world_pose(1).isApprox(third_at * fourth_at, chain_tolerance));
}
