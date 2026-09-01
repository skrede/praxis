#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/frame_stencil.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <threepp/objects/Group.hpp>
#include <threepp/scenes/Scene.hpp>
#include <threepp/core/Object3D.hpp>

#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>

using namespace praxis;
using namespace praxis::rigid_motion;

namespace {

constexpr double place_tolerance = 1.0e-6;

std::size_t descendants(threepp::Object3D &root)
{
    std::size_t counted = 0;
    root.traverse([&counted](threepp::Object3D &) { ++counted; });

    return counted;
}

stencil_object described(std::string name, bool with_axes, body_shape shape)
{
    stencil_object object;
    object.name       = std::move(name);
    object.axes.shown = with_axes;
    object.body.shape = shape;
    if(shape == body_shape::mesh)
        object.body.mesh = threepp::Group::create();

    return object;
}

transform placed_at(double x, double y, double z)
{
    transform tf         = transform::Identity();
    tf.block<3, 1>(0, 3) = Eigen::Vector3d{x, y, z};

    return tf;
}

threepp::Vector3 world_position(threepp::Object3D &node)
{
    threepp::Vector3 put;
    node.getWorldPosition(put);

    return put;
}

// The renderer stores a node's placement in single precision, so a metre written in double reads
// back within a float's resolution of it rather than exactly.
bool same_place(const threepp::Vector3 &put, const threepp::Vector3 &expected)
{
    return put.x == Catch::Approx(expected.x).margin(place_tolerance) && put.y == Catch::Approx(expected.y).margin(place_tolerance) &&
            put.z == Catch::Approx(expected.z).margin(place_tolerance);
}

}

TEST_CASE("the stencil puts one root into the scene and takes the same one back out")
{
    threepp::Scene target;
    const std::size_t empty = descendants(target);

    frame_stencil body(target, std::vector<stencil_object>{described("Frame", true, body_shape::cube)}, frame_ops{});
    REQUIRE(descendants(target) == empty);

    REQUIRE(body.initialize().has_value());
    REQUIRE(descendants(target) > empty);
    REQUIRE(target.getObjectByName("Frame") != nullptr);

    body.tear_down();
    REQUIRE(descendants(target) == empty);
    REQUIRE(target.getObjectByName("Frame") == nullptr);
}

TEST_CASE("one object and three objects are the same type")
{
    threepp::Scene target;

    frame_stencil alone(target, std::vector<stencil_object>{described("Only", true, body_shape::cube)}, frame_ops{});
    REQUIRE(alone.count() == 1);
    REQUIRE(alone.name_of(0) == "Only");

    frame_stencil several(
            target, std::vector<stencil_object>{described("First", true, body_shape::cube), described("Second", true, body_shape::cube), described("Third", true, body_shape::cube)},
            frame_ops{});
    REQUIRE(several.count() == 3);
    REQUIRE(several.name_of(0) == "First");
    REQUIRE(several.name_of(1) == "Second");
    REQUIRE(several.name_of(2) == "Third");

    REQUIRE(several.initialize().has_value());
    REQUIRE(target.getObjectByName("First") != nullptr);
    REQUIRE(target.getObjectByName("Second") != nullptr);
    REQUIRE(target.getObjectByName("Third") != nullptr);
}

TEST_CASE("moving one object moves that object and no other")
{
    threepp::Scene target;
    frame_stencil body(target,
                       std::vector<stencil_object>{described("First", true, body_shape::cube), described("Second", true, body_shape::cube), described("Third", true, body_shape::cube)},
                       frame_ops{});
    REQUIRE(body.initialize().has_value());
    body.render();

    threepp::Object3D *const first  = target.getObjectByName("First");
    threepp::Object3D *const second = target.getObjectByName("Second");
    threepp::Object3D *const third  = target.getObjectByName("Third");
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);
    REQUIRE(third != nullptr);

    const threepp::Vector3 second_before = world_position(*second);
    const threepp::Vector3 third_before  = world_position(*third);

    body.set_pose(1, placed_at(0.4, -0.2, 1.1));
    body.render();

    // The root carries the z-up quarter turn, so a pose written in z reads back on the renderer's y.
    REQUIRE(same_place(world_position(*second), threepp::Vector3{0.4f, 1.1f, 0.2f}));

    REQUIRE(same_place(world_position(*third), third_before));
    REQUIRE(same_place(world_position(*first), threepp::Vector3{0.f, 0.f, 0.f}));
    REQUIRE(same_place(second_before, threepp::Vector3{0.f, 0.f, 0.f}));

    REQUIRE(body.pose(1)(0, 3) == Catch::Approx(0.4));
    REQUIRE(body.pose(0).isApprox(transform::Identity()));
}

TEST_CASE("axes with no body render the origin marker")
{
    threepp::Scene target;
    frame_stencil body(target, std::vector<stencil_object>{described("Bare", true, body_shape::none)}, frame_ops{});
    REQUIRE(body.initialize().has_value());

    threepp::Object3D *const node = target.getObjectByName("Bare");
    REQUIRE(node != nullptr);
    REQUIRE(node->getObjectByName("axes") != nullptr);
    REQUIRE(node->getObjectByName("origin") != nullptr);
    REQUIRE(node->getObjectByName("body") == nullptr);
    REQUIRE(descendants(*node) == 12);
}

TEST_CASE("axes with a body render the body in the marker's place")
{
    threepp::Scene target;
    frame_stencil body(target, std::vector<stencil_object>{described("Both", true, body_shape::cube)}, frame_ops{});
    REQUIRE(body.initialize().has_value());

    threepp::Object3D *const node = target.getObjectByName("Both");
    REQUIRE(node != nullptr);
    REQUIRE(node->getObjectByName("axes") != nullptr);
    REQUIRE(node->getObjectByName("origin") == nullptr);
    REQUIRE(node->getObjectByName("body") != nullptr);
    REQUIRE(descendants(*node) == 12);
}

TEST_CASE("a body with axes hidden hangs them all the same and draws the body alone")
{
    threepp::Scene target;
    frame_stencil body(target, std::vector<stencil_object>{described("Blind", false, body_shape::cube)}, frame_ops{});
    REQUIRE(body.initialize().has_value());

    threepp::Object3D *const node = target.getObjectByName("Blind");
    REQUIRE(node != nullptr);
    REQUIRE(node->getObjectByName("axes") != nullptr);
    REQUIRE_FALSE(node->getObjectByName("axes")->visible);
    REQUIRE(node->getObjectByName("origin") == nullptr);
    REQUIRE(node->getObjectByName("body") != nullptr);
    REQUIRE(descendants(*node) == 12);
}

TEST_CASE("an object with its axes hidden and no body is a pose and nothing on screen")
{
    threepp::Scene target;
    frame_stencil body(target, std::vector<stencil_object>{described("Empty", false, body_shape::none)}, frame_ops{});
    REQUIRE(body.initialize().has_value());

    threepp::Object3D *const node = target.getObjectByName("Empty");
    REQUIRE(node != nullptr);
    REQUIRE_FALSE(node->getObjectByName("axes")->visible);
    REQUIRE(node->getObjectByName("origin") != nullptr);
    REQUIRE(descendants(*node) == 12);

    body.set_pose(0, placed_at(0.0, 0.0, 0.7));
    body.render();
    REQUIRE(same_place(world_position(*node), threepp::Vector3{0.f, 0.7f, 0.f}));
}

TEST_CASE("hiding and showing an object's axes leaves the scene the same size")
{
    threepp::Scene target;
    frame_stencil body(target, std::vector<stencil_object>{described("Frame", true, body_shape::cube), described("Attached", true, body_shape::none)}, frame_ops{});
    REQUIRE(body.initialize().has_value());
    REQUIRE(body.set_parent(1, std::size_t{0}).has_value());
    body.set_pose(1, placed_at(0.3, 0.0, 0.0));

    threepp::Object3D *const node    = target.getObjectByName("Attached");
    const std::size_t hung           = descendants(*node);
    const std::size_t hung_elsewhere = descendants(*target.getObjectByName("Frame"));

    REQUIRE(body.axes_shown(1));

    body.set_axes_shown(1, false);
    REQUIRE_FALSE(body.axes_shown(1));
    REQUIRE_FALSE(node->getObjectByName("axes")->visible);
    REQUIRE(descendants(*node) == hung);

    body.set_axes_shown(1, true);
    REQUIRE(body.axes_shown(1));
    REQUIRE(node->getObjectByName("axes")->visible);
    REQUIRE(descendants(*node) == hung);

    // The object beside it is untouched, and so is everything the toggle is not a view of.
    REQUIRE(body.axes_shown(0));
    REQUIRE(descendants(*target.getObjectByName("Frame")) == hung_elsewhere);
    REQUIRE(body.pose(1).isApprox(placed_at(0.3, 0.0, 0.0)));
    REQUIRE(body.parent_of(1) == std::optional<std::size_t>(std::size_t{0}));
    REQUIRE(body.name_of(1) == "Attached");
}

TEST_CASE("a mesh body is drawn where a cube would be")
{
    threepp::Scene target;
    frame_stencil body(target, std::vector<stencil_object>{described("Loaded", true, body_shape::mesh)}, frame_ops{});
    REQUIRE(body.initialize().has_value());

    threepp::Object3D *const node = target.getObjectByName("Loaded");
    REQUIRE(node != nullptr);
    REQUIRE(node->getObjectByName("body") != nullptr);
    REQUIRE(node->getObjectByName("origin") == nullptr);
    REQUIRE(descendants(*node) == 13);
}

TEST_CASE("replacing a body leaves the pose and the axes where they were")
{
    threepp::Scene target;
    frame_stencil body(target, std::vector<stencil_object>{described("Frame", true, body_shape::cube)}, frame_ops{});
    REQUIRE(body.initialize().has_value());
    body.set_pose(0, placed_at(0.1, 0.2, 0.3));
    body.render();

    threepp::Object3D *const node        = target.getObjectByName("Frame");
    const threepp::Object3D *const drawn = node->getObjectByName("axes");
    const std::size_t axes_before        = descendants(*node->getObjectByName("axes"));
    const threepp::Vector3 placed        = world_position(*node);

    body.set_body(0, object_body{body_shape::cube, 0.5, nullptr});
    body.render();

    REQUIRE(node->getObjectByName("axes") == drawn);
    REQUIRE(descendants(*node->getObjectByName("axes")) == axes_before);
    REQUIRE(node->getObjectByName("origin") == nullptr);
    REQUIRE(same_place(world_position(*node), placed));
    REQUIRE(body.pose(0).isApprox(placed_at(0.1, 0.2, 0.3)));
    REQUIRE(descendants(*node) == 12);
}

TEST_CASE("taking a body away brings the origin marker back")
{
    threepp::Scene target;
    frame_stencil body(target, std::vector<stencil_object>{described("Frame", true, body_shape::cube)}, frame_ops{});
    REQUIRE(body.initialize().has_value());

    threepp::Object3D *const node = target.getObjectByName("Frame");
    REQUIRE(node->getObjectByName("origin") == nullptr);

    body.set_body(0, object_body{body_shape::none, 0.0, nullptr});
    REQUIRE(node->getObjectByName("body") == nullptr);
    REQUIRE(node->getObjectByName("origin") != nullptr);
    REQUIRE(descendants(*node) == 12);

    body.set_body(0, object_body{body_shape::cube, 0.25, nullptr});
    REQUIRE(node->getObjectByName("body") != nullptr);
    REQUIRE(node->getObjectByName("origin") == nullptr);
    REQUIRE(descendants(*node) == 12);
}

TEST_CASE("an index past the end changes nothing and answers with the identity")
{
    threepp::Scene target;
    frame_stencil body(target, std::vector<stencil_object>{described("Frame", true, body_shape::cube)}, frame_ops{});
    REQUIRE(body.initialize().has_value());
    body.set_pose(0, placed_at(0.1, 0.2, 0.3));

    threepp::Object3D *const node  = target.getObjectByName("Frame");
    const std::size_t drawn_before = descendants(*node);

    body.set_pose(9, placed_at(5.0, 5.0, 5.0));
    body.set_body(9, object_body{body_shape::none, 0.0, nullptr});
    body.set_axes_shown(9, false);

    REQUIRE(body.count() == 1);
    REQUIRE(body.pose(9).isApprox(transform::Identity()));
    REQUIRE(body.name_of(9).empty());
    REQUIRE_FALSE(body.axes_shown(9));
    REQUIRE(body.axes_shown(0));
    REQUIRE(body.pose(0).isApprox(placed_at(0.1, 0.2, 0.3)));
    REQUIRE(descendants(*node) == drawn_before);
}

TEST_CASE("naming the fixed frame hangs its node in the scene, and naming none hangs nothing")
{
    threepp::Scene plain;
    frame_stencil unnamed(plain, std::vector<stencil_object>{described("Frame", true, body_shape::cube)}, frame_ops{});
    REQUIRE(unnamed.initialize().has_value());

    threepp::Scene target;
    frame_stencil body(target, std::vector<stencil_object>{described("Frame", true, body_shape::cube)}, frame_ops{}, fixed_frame{"Ground", axes_settings{}});
    REQUIRE(body.initialize().has_value());

    threepp::Object3D *const node = target.getObjectByName("Ground");
    REQUIRE(node != nullptr);
    REQUIRE(body.fixed_frame_name() == "Ground");

    REQUIRE(unnamed.fixed_frame_name().empty());
    REQUIRE(plain.getObjectByName("Ground") == nullptr);
    REQUIRE(descendants(target) == descendants(plain) + descendants(*node));

    body.tear_down();
    REQUIRE(target.getObjectByName("Ground") == nullptr);
}

TEST_CASE("the fixed frame stands at the scene origin and no object moves it")
{
    threepp::Scene target;
    frame_stencil body(target, std::vector<stencil_object>{described("Frame", true, body_shape::cube)}, frame_ops{}, fixed_frame{"Ground", axes_settings{}});
    REQUIRE(body.initialize().has_value());

    threepp::Object3D *const node = target.getObjectByName("Ground");
    REQUIRE(node != nullptr);
    REQUIRE(same_place(world_position(*node), threepp::Vector3{0.f, 0.f, 0.f}));

    body.set_pose(0, placed_at(0.4, -0.2, 1.1));
    body.render();

    REQUIRE(same_place(world_position(*node), threepp::Vector3{0.f, 0.f, 0.f}));
    REQUIRE(same_place(world_position(*target.getObjectByName("Frame")), threepp::Vector3{0.4f, 1.1f, 0.2f}));
}

TEST_CASE("the fixed frame is no object of the stencil")
{
    threepp::Scene plain;
    frame_stencil unnamed(plain, std::vector<stencil_object>{described("Frame", true, body_shape::cube), described("Second", true, body_shape::cube)}, frame_ops{});

    threepp::Scene target;
    frame_stencil body(target, std::vector<stencil_object>{described("Frame", true, body_shape::cube), described("Second", true, body_shape::cube)}, frame_ops{},
                       fixed_frame{"Ground", axes_settings{}});
    REQUIRE(body.initialize().has_value());

    REQUIRE(body.count() == unnamed.count());
    for(std::size_t index = 0; index < body.count(); ++index)
        REQUIRE(body.name_of(index) != "Ground");

    REQUIRE(body.remove(1).has_value());
    REQUIRE(body.remove(0).has_value());

    REQUIRE(body.count() == 0);
    REQUIRE(target.getObjectByName("Frame") == nullptr);
    REQUIRE(target.getObjectByName("Ground") != nullptr);
    REQUIRE(body.fixed_frame_name() == "Ground");
}

TEST_CASE("the fixed frame is drawn or hidden as the composition described it")
{
    axes_settings hidden;
    hidden.shown = false;

    threepp::Scene target;
    frame_stencil drawn(target, std::vector<stencil_object>{}, frame_ops{}, fixed_frame{"Shown", axes_settings{}});
    REQUIRE(drawn.initialize().has_value());

    threepp::Scene elsewhere;
    frame_stencil unseen(elsewhere, std::vector<stencil_object>{}, frame_ops{}, fixed_frame{"Hidden", hidden});
    REQUIRE(unseen.initialize().has_value());

    threepp::Object3D *const first  = target.getObjectByName("Shown");
    threepp::Object3D *const second = elsewhere.getObjectByName("Hidden");
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);

    REQUIRE(first->visible);
    REQUIRE_FALSE(second->visible);
    REQUIRE(descendants(*first) == descendants(*second));
}
