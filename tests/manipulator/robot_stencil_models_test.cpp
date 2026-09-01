#include "fixtures.h"

#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <memory>
#include <cstddef>
#include <stdexcept>

using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

time_point dictated{};

time_point reading()
{
    return dictated;
}

clock_source dictating()
{
    dictated = time_point{};
    return clock_source{&reading};
}

// Only the configuration is read by what is under test, but a snapshot carries the whole readable
// state and has no field a publication may leave out, so the rest is filled with what an arm at rest
// reports.
arm_snapshot at_rest(const joint_vector &joints)
{
    const praxis::transform identity = praxis::transform::Identity();
    const Eigen::Vector3d origin(Eigen::Vector3d::Zero());
    const praxis::rotation upright(praxis::rotation::Identity());

    return arm_snapshot{joints,
                        joint_limits{},
                        identity,
                        identity,
                        identity,
                        origin,
                        origin,
                        upright,
                        upright,
                        recording_parameters{},
                        1.0,
                        false,
                        task_counters{},
                        {},
                        praxis::unexpected(praxis::refusal::not_implemented),
                        praxis::unexpected(praxis::refusal::not_implemented),
                        jacobian_manipulability{praxis::unexpected(praxis::refusal::not_implemented), praxis::unexpected(praxis::refusal::not_implemented)},
                        jacobian_manipulability{praxis::unexpected(praxis::refusal::not_implemented), praxis::unexpected(praxis::refusal::not_implemented)},
                        {},
                        {},
                        nullptr,
                        nullptr,
                        {},
                        {}};
}

// What the composing side hands over is an object that already exists, and nothing under test reads
// its geometry, so an empty node stands in for a mesh a loader would have produced.
std::shared_ptr<threepp::Object3D> supplied_model()
{
    return threepp::Object3D::create();
}

std::size_t descendants(threepp::Object3D &root)
{
    std::size_t counted = 0;
    root.traverse([&counted](threepp::Object3D &) { ++counted; });

    return counted;
}

std::shared_ptr<arm_publisher> publishing(const joint_vector &joints)
{
    auto published = std::make_shared<arm_publisher>();
    published->publish(std::make_shared<const arm_snapshot>(at_rest(joints)));

    return published;
}

}

TEST_CASE("a stencil built with a tool and a world reference object holds both in the scene and leaves neither behind", "[manipulator][stencil]")
{
    scheduler loop(inline_workers, dictating());
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    const auto published                         = publishing(configuration(0.25, -0.5));
    const std::size_t bare                       = descendants(*target);

    const std::shared_ptr<threepp::Object3D> tool  = supplied_model();
    const std::shared_ptr<threepp::Object3D> world = supplied_model();

    loadable_robot_stencil shown(two_joint_handle(), attached_models{tool, world}, *target, loop.main_strand(), published->reader(), praxis::rigid_motion::baseline().screw,
                                 praxis::rigid_motion::screw_slot_set{});

    REQUIRE(shown.initialize().has_value());
    CHECK(shown.attached_at(flange_attachment::tool) == tool);
    CHECK(shown.world_object() == world);
    CHECK(tool->parent == target.get());
    CHECK(world->parent == target.get());
    CHECK(descendants(*target) > bare);

    shown.tear_down();

    CHECK(tool->parent == nullptr);
    CHECK(world->parent == nullptr);
    CHECK(descendants(*target) == bare);
}

// The third model is the one a composition may have nothing to give: its absence is what the stencil
// is asked to carry, rather than an object with nothing in it.
TEST_CASE("a composition supplying no world reference object composes, initializes, renders and tears down", "[manipulator][stencil]")
{
    scheduler loop(inline_workers, dictating());
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    const auto published                         = publishing(configuration(0.25, -0.5));
    const std::size_t bare                       = descendants(*target);

    const std::shared_ptr<threepp::Object3D> tool = supplied_model();

    loadable_robot_stencil shown(two_joint_handle(), attached_models{tool, nullptr}, *target, loop.main_strand(), published->reader(), praxis::rigid_motion::baseline().screw,
                                 praxis::rigid_motion::screw_slot_set{});

    REQUIRE(shown.initialize().has_value());
    CHECK(shown.world_object() == nullptr);
    CHECK(tool->parent == target.get());

    REQUIRE(loop.main_strand().post([&shown] { shown.render(); }).has_value());
    REQUIRE(loop.drain().has_value());

    shown.tear_down();

    CHECK(descendants(*target) == bare);
}

TEST_CASE("a stencil built with neither model carries nothing at the flange until an attachment is installed", "[manipulator][stencil]")
{
    scheduler loop(inline_workers, dictating());
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    const auto published                         = publishing(configuration(0.0, 0.0));

    loadable_robot_stencil shown(two_joint_handle(), attached_models{}, *target, loop.main_strand(), published->reader(), praxis::rigid_motion::baseline().screw,
                                 praxis::rigid_motion::screw_slot_set{});

    REQUIRE(shown.initialize().has_value());
    CHECK(shown.attached_at(flange_attachment::tool) == nullptr);
    CHECK(shown.attached_at(flange_attachment::frame_marker) == nullptr);
    CHECK(shown.world_object() == nullptr);

    const auto axes = threepp::AxesHelper::create(0.1f);
    shown.set_flange_attachment(flange_attachment::tool, axes, threepp::Matrix4().makeRotationY(-threepp::math::PI / 2.f));

    CHECK(shown.attached_at(flange_attachment::tool) == axes);
    CHECK(axes->parent == target.get());

    shown.tear_down();
}

TEST_CASE("setting a tool at run time detaches the object the stencil was built with", "[manipulator][stencil]")
{
    scheduler loop(inline_workers, dictating());
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    const auto published                         = publishing(configuration(0.0, 0.0));

    const std::shared_ptr<threepp::Object3D> supplied = supplied_model();
    const std::shared_ptr<threepp::Object3D> swapped  = supplied_model();

    loadable_robot_stencil shown(two_joint_handle(), attached_models{supplied, nullptr}, *target, loop.main_strand(), published->reader(), praxis::rigid_motion::baseline().screw,
                                 praxis::rigid_motion::screw_slot_set{});

    REQUIRE(shown.initialize().has_value());
    REQUIRE(supplied->parent == target.get());

    shown.set_flange_attachment(flange_attachment::tool, swapped);

    CHECK(shown.attached_at(flange_attachment::tool) == swapped);
    CHECK(supplied->parent == nullptr);
    CHECK(swapped->parent == target.get());

    shown.tear_down();
}

TEST_CASE("a stencil built with no robot object refuses rather than reading it", "[manipulator][stencil]")
{
    scheduler loop(inline_workers, dictating());
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();
    const auto published                         = publishing(configuration(0.0, 0.0));
    const std::size_t bare                       = descendants(*target);

    CHECK_THROWS_AS(loadable_robot_stencil(nullptr, attached_models{supplied_model(), supplied_model()}, *target, loop.main_strand(), published->reader(),
                                           praxis::rigid_motion::baseline().screw, praxis::rigid_motion::screw_slot_set{}),
                    std::invalid_argument);
    CHECK(descendants(*target) == bare);
}
