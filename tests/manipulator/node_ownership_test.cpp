#include "fixtures.h"

#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/evaluation/tolerance.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <memory>
#include <string>
#include <cstdint>
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

// Everything a frame needs to reach the node: the scene it is added to, the publication it mirrors,
// and the strand that owns it. A scene is created headlessly and a renderer robot needs no graphics
// context, so no display is involved.
struct mirror
{
    explicit mirror(scheduler &loop)
            : scene(threepp::Scene::create())
            , published(std::make_shared<arm_publisher>())
            , stencil(two_joint_handle(), attached_models{}, *scene, loop.main_strand(), published->reader(), praxis::rigid_motion::baseline().screw,
                      praxis::rigid_motion::screw_slot_set{})
    {
    }

    void publish(const joint_vector &joints)
    {
        published->publish(std::make_shared<const arm_snapshot>(at_rest(joints)));
    }

    double node_joint(unsigned index)
    {
        return static_cast<double>(stencil.robot().getJointValue(index));
    }

    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<arm_publisher> published;
    loadable_robot_stencil stencil;
};

// A handler that lets an exception out does not hand it back to whoever drained, so the refusal is
// caught where it is thrown and what the case reads is the message it carried.
std::string refused_rendering_on(scheduler &loop, const strand &on, const loadable_robot_stencil &shown)
{
    std::string reported;

    REQUIRE(on.post(
                      [&]
                      {
                          try
                          {
                              shown.render();
                          }
                          catch(const std::logic_error &wrong_strand)
                          {
                              reported = wrong_strand.what();
                          }
                      })
                    .has_value());
    REQUIRE(loop.drain().has_value());

    return reported;
}

}

TEST_CASE("rendering the arm from a strand that does not own the node throws and names both", "[manipulator][ownership]")
{
    scheduler loop(inline_workers, dictating());
    mirror shown(loop);
    const strand elsewhere = *loop.make_strand();

    shown.publish(configuration(0.25, -0.5));

    const std::string reported = refused_rendering_on(loop, elsewhere, shown.stencil);

    REQUIRE_FALSE(reported.empty());
    REQUIRE(reported.find("the rendered robot") != std::string::npos);
    REQUIRE(reported.find("strand " + std::to_string(static_cast<std::uint32_t>(loop.main_strand().id()))) != std::string::npos);
}

TEST_CASE("rendering the arm from the strand that owns the node writes the published configuration", "[manipulator][ownership]")
{
    scheduler loop(inline_workers, dictating());
    mirror shown(loop);

    shown.publish(configuration(0.25, -0.5));

    const std::string reported = refused_rendering_on(loop, loop.main_strand(), shown.stencil);

    REQUIRE(reported.empty());
    CHECK(praxis::is_approx_equal(shown.node_joint(0u), 0.25, round_trip));
    CHECK(praxis::is_approx_equal(shown.node_joint(1u), -0.5, round_trip));
}

// A publication the frame did not run for is never drawn, which is the mirror's whole contract: the
// display cannot show a configuration between two frames, so the node carries the latest one only.
TEST_CASE("a publication arriving between two frames is applied on the second and not the first", "[manipulator][ownership]")
{
    scheduler loop(inline_workers, dictating());
    mirror shown(loop);

    shown.publish(configuration(0.25, -0.5));
    REQUIRE(refused_rendering_on(loop, loop.main_strand(), shown.stencil).empty());

    CHECK(praxis::is_approx_equal(shown.node_joint(0u), 0.25, round_trip));

    shown.publish(configuration(-0.75, 1.25));
    CHECK(praxis::is_approx_equal(shown.node_joint(0u), 0.25, round_trip));

    REQUIRE(refused_rendering_on(loop, loop.main_strand(), shown.stencil).empty());

    CHECK(praxis::is_approx_equal(shown.node_joint(0u), -0.75, round_trip));
    CHECK(praxis::is_approx_equal(shown.node_joint(1u), 1.25, round_trip));
}
