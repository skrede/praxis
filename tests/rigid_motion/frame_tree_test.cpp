#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/frame_tree.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/frame_stencil.h"

#include "praxis/compat/expected.h"

#include "praxis/extension/refusal.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>
#include <threepp/core/Object3D.hpp>

#include <Eigen/Geometry>

#include <vector>
#include <cstddef>
#include <optional>

using namespace praxis;
using namespace praxis::rigid_motion;

namespace {

constexpr double chain_tolerance = 1.0e-9;
constexpr float place_tolerance  = 1.0e-6f;

transform turned(double radians, const Eigen::Vector3d &at)
{
    transform tf         = transform::Identity();
    tf.block<3, 3>(0, 0) = Eigen::AngleAxisd(radians, Eigen::Vector3d{1.0, -2.0, 3.0}.normalized()).toRotationMatrix();
    tf.block<3, 1>(0, 3) = at;

    return tf;
}

const transform first  = turned(0.4, Eigen::Vector3d{0.3, -0.2, 0.9});
const transform second = turned(-0.9, Eigen::Vector3d{1.1, 0.5, -0.4});
const transform third  = turned(1.7, Eigen::Vector3d{-0.6, 0.8, 0.2});

frame_tree chained(frame_ops motions)
{
    frame_tree arranged(3, motions);
    arranged.set_pose(0, first);
    arranged.set_pose(1, second);
    arranged.set_pose(2, third);
    REQUIRE(arranged.set_parent(1, std::size_t{0}).has_value());
    REQUIRE(arranged.set_parent(2, std::size_t{1}).has_value());

    return arranged;
}

// The renderer stores a node's placement in single precision, so a metre written in double reads
// back within a float's resolution of it rather than exactly.
float away_from(threepp::Object3D &node, const threepp::Vector3 &expected)
{
    threepp::Vector3 put;
    node.getWorldPosition(put);

    return put.distanceTo(expected);
}

}

TEST_CASE("a chain composes each frame's placement in the one above it, and a frame with no parent reads the same both ways")
{
    const frame_tree arranged = chained(baseline().frame);

    REQUIRE(arranged.count() == 3);
    REQUIRE(arranged.parent_of(0) == std::nullopt);
    REQUIRE(arranged.parent_of(2) == std::optional<std::size_t>{1});
    REQUIRE(arranged.pose(2).isApprox(third));
    REQUIRE(arranged.world_pose(0).isApprox(first, chain_tolerance));
    REQUIRE(arranged.world_pose(1).isApprox(first * second, chain_tolerance));
    REQUIRE(arranged.world_pose(2).isApprox(first * second * third, chain_tolerance));

    REQUIRE(arranged.world_pose(0).isApprox(arranged.pose(0), chain_tolerance));
    REQUIRE_FALSE(arranged.world_pose(1).isApprox(arranged.pose(1), chain_tolerance));
    REQUIRE_FALSE(arranged.world_pose(2).isApprox(arranged.pose(2), chain_tolerance));
}

TEST_CASE("a parent that would close a cycle is refused and the relation is left as it was")
{
    frame_tree arranged = chained(baseline().frame);

    SECTION("a frame's own descendant")
    {
        const expected<void, refusal> refused = arranged.set_parent(0, std::size_t{2});
        REQUIRE_FALSE(refused.has_value());
        REQUIRE(refused.error() == refusal::unsupported_input);
        REQUIRE(arranged.parent_of(0) == std::nullopt);
        REQUIRE(arranged.world_pose(2).isApprox(first * second * third, chain_tolerance));
    }

    SECTION("a frame itself")
    {
        const expected<void, refusal> refused = arranged.set_parent(1, std::size_t{1});
        REQUIRE_FALSE(refused.has_value());
        REQUIRE(refused.error() == refusal::unsupported_input);
        REQUIRE(arranged.parent_of(1) == std::optional<std::size_t>{0});
    }

    SECTION("a frame past the end")
    {
        const expected<void, refusal> named = arranged.set_parent(2, std::size_t{9});
        const expected<void, refusal> child = arranged.set_parent(9, std::size_t{0});
        REQUIRE(named.error() == refusal::unsupported_input);
        REQUIRE(child.error() == refusal::unsupported_input);
        REQUIRE(arranged.parent_of(2) == std::optional<std::size_t>{1});
        REQUIRE(arranged.parent_of(9) == std::nullopt);
    }
}

TEST_CASE("moving a frame carries its descendants and no other frame")
{
    frame_tree arranged   = chained(baseline().frame);
    const transform moved = turned(0.25, Eigen::Vector3d{2.0, 0.0, -1.0});

    arranged.set_pose(1, moved);

    REQUIRE(arranged.world_pose(0).isApprox(first, chain_tolerance));
    REQUIRE(arranged.world_pose(1).isApprox(first * moved, chain_tolerance));
    REQUIRE(arranged.world_pose(2).isApprox(first * moved * third, chain_tolerance));
}

TEST_CASE("the stencil renders the composed pose and keeps its nodes flat")
{
    threepp::Scene target;
    frame_stencil body(target, std::vector<stencil_object>{stencil_object{"Frame", axes_settings{}, object_body{}}, stencil_object{"Attached", axes_settings{}, object_body{}}},
                       baseline().frame);
    REQUIRE(body.initialize().has_value());

    body.set_pose(0, turned(0.0, Eigen::Vector3d{0.0, 0.0, 0.5}));
    body.set_pose(1, turned(0.0, Eigen::Vector3d{0.4, 0.0, 0.0}));
    REQUIRE(body.set_parent(1, std::size_t{0}).has_value());
    body.render();

    threepp::Object3D *const carried  = target.getObjectByName("Frame");
    threepp::Object3D *const attached = target.getObjectByName("Attached");
    REQUIRE(carried != nullptr);
    REQUIRE(attached != nullptr);
    REQUIRE(body.pose(1).isApprox(turned(0.0, Eigen::Vector3d{0.4, 0.0, 0.0})));
    REQUIRE(body.world_pose(1).isApprox(turned(0.0, Eigen::Vector3d{0.4, 0.0, 0.5}), chain_tolerance));

    // The root carries the z-up quarter turn, so a pose written in z reads back on the renderer's y.
    REQUIRE(away_from(*attached, threepp::Vector3{0.4f, 0.5f, 0.f}) < place_tolerance);
    REQUIRE(away_from(*carried, threepp::Vector3{0.f, 0.5f, 0.f}) < place_tolerance);
    REQUIRE(attached->parent == carried->parent);
}
