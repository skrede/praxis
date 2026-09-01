#ifndef HPP_GUARD_PRAXIS_TESTS_RIGID_MOTION_MARKED_STENCIL_H
#define HPP_GUARD_PRAXIS_TESTS_RIGID_MOTION_MARKED_STENCIL_H

#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/frame_stencil.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <threepp/objects/Mesh.hpp>
#include <threepp/objects/Group.hpp>

#include <threepp/math/Box3.hpp>
#include <threepp/math/Color.hpp>
#include <threepp/math/Vector3.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/core/Object3D.hpp>

#include <threepp/geometries/BoxGeometry.hpp>

#include <threepp/materials/MeshPhongMaterial.hpp>

#include <string>
#include <memory>
#include <vector>
#include <cstddef>
#include <utility>

namespace praxis::fixture {

inline constexpr double place_tolerance = 1.0e-6;

inline constexpr float loaded_edge = 0.2f;

inline rigid_motion::stencil_object described(std::string name, bool with_axes, rigid_motion::body_shape shape)
{
    rigid_motion::stencil_object object;
    object.name       = std::move(name);
    object.axes.shown = with_axes;
    object.body.shape = shape;

    return object;
}

// A body standing for one a caller loaded from disk: the stencil wraps whatever object it is handed
// and never reaches inside it.
inline std::shared_ptr<threepp::Object3D> loaded_body()
{
    auto carried = threepp::Group::create();
    carried->add(threepp::Mesh::create(threepp::BoxGeometry::create(loaded_edge, loaded_edge, loaded_edge)));

    return carried;
}

inline transform placed_at(double x, double y, double z)
{
    transform tf         = transform::Identity();
    tf.block<3, 1>(0, 3) = Eigen::Vector3d{x, y, z};

    return tf;
}

inline threepp::Object3D &node_named(threepp::Scene &target, const char *name)
{
    threepp::Object3D *const found = target.getObjectByName(name);
    REQUIRE(found != nullptr);

    return *found;
}

// The mark composes its own placement inside the world-matrix update the renderer gives every node,
// and reading a node's placement does not go through that update, so the scene is brought up to date
// before the mark is read.
inline threepp::Object3D &mark_in(threepp::Scene &target, const rigid_motion::frame_stencil &body)
{
    body.render();
    target.updateMatrixWorld(true);

    return node_named(target, "mark");
}

// Where a node stands in the renderer's world, which is the frame the extent a mark is taken into is
// expressed in. A node's own placement is read in its parent's frame and says nothing about where it
// hangs.
inline threepp::Vector3 world_place(threepp::Object3D &node)
{
    threepp::Vector3 put;
    node.getWorldPosition(put);

    return put;
}

inline threepp::Box3 extent_of(threepp::Object3D &node)
{
    threepp::Box3 around;
    around.setFromObject(node);

    return around;
}

inline threepp::Color drawn_tone(threepp::Scene &target, const char *name)
{
    threepp::Object3D *const body = node_named(target, name).getObjectByName("body");
    REQUIRE(body != nullptr);

    auto *const lit = body->materialAs<threepp::MeshPhongMaterial>();
    REQUIRE(lit != nullptr);

    return lit->color;
}

// The renderer stores a placement in single precision, so a metre written in double reads back
// within a float's resolution of it rather than exactly.
inline bool same_place(const threepp::Vector3 &put, const threepp::Vector3 &expected)
{
    return put.x == Catch::Approx(expected.x).margin(place_tolerance) && put.y == Catch::Approx(expected.y).margin(place_tolerance) &&
            put.z == Catch::Approx(expected.z).margin(place_tolerance);
}

inline std::vector<rigid_motion::stencil_object> two_cubes()
{
    return std::vector<rigid_motion::stencil_object>{described("First", true, rigid_motion::body_shape::cube), described("Second", true, rigid_motion::body_shape::cube)};
}

inline std::vector<rigid_motion::stencil_object> one(rigid_motion::stencil_object alone)
{
    return std::vector<rigid_motion::stencil_object>{std::move(alone)};
}

// One described object beside a plain companion, for a case about that object's own kind: the set is
// large enough to be marked at all, and the described one stands first, where a selection opens.
inline std::vector<rigid_motion::stencil_object> accompanied(rigid_motion::stencil_object about)
{
    return std::vector<rigid_motion::stencil_object>{std::move(about), described("Companion", true, rigid_motion::body_shape::cube)};
}

// One stencil over a scene of its own, standing on the frame whose index the case writes. The
// selection is the shape a composition hands the stencil: one callable answered afresh every frame.
struct marked_scene
{
    explicit marked_scene(std::vector<rigid_motion::stencil_object> objects, rigid_motion::fixed_frame anchored = rigid_motion::fixed_frame())
            : chosen(0)
            , stage()
            , body(stage, std::move(objects), rigid_motion::frame_ops{}, std::move(anchored))
    {
        body.follow_selection([this] { return chosen; });
        REQUIRE(body.initialize().has_value());
    }

    marked_scene(const marked_scene &) = delete;

    threepp::Object3D &mark()
    {
        return mark_in(stage, body);
    }

    std::size_t chosen;

    // Declared ahead of the stencil so that it is destroyed after one: the stencil holds this scene
    // for as long as it lives.
    threepp::Scene stage;
    rigid_motion::frame_stencil body;
};

}

#endif
