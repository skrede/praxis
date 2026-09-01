#include "fixtures.h"
#include "drawn_chain.h"
#include "captured_log.h"

#include "../presets/drawn_lines.h"

#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>

using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

// Only the configuration is read by what is under test, but a snapshot has no field a publication
// may leave out, so the rest is filled with what an arm at rest reports.
arm_snapshot at_rest(const joint_vector &joints)
{
    const praxis::transform put = praxis::transform::Identity();
    const Eigen::Vector3d origin(Eigen::Vector3d::Zero());
    const praxis::rotation upright(praxis::rotation::Identity());

    return arm_snapshot{joints,
                        joint_limits{},
                        put,
                        put,
                        put,
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

std::vector<praxis::screw_axis> two_axes()
{
    const praxis::rigid_motion::screw_ops screw = praxis::rigid_motion::baseline().screw;

    return {screw.screw_axis_from_point_direction_pitch(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), 0.0).value(),
            screw.screw_axis_from_point_direction_pitch(Eigen::Vector3d(static_cast<double>(link_length), 0.0, 0.0), Eigen::Vector3d::UnitZ(), 0.0).value()};
}

std::vector<praxis::transform> two_poses()
{
    praxis::transform first  = praxis::transform::Identity();
    praxis::transform second = praxis::transform::Identity();
    first(0, 3)              = 0.25;
    second(1, 3)             = 0.5;

    return {first, second};
}

// A scene needs no graphics context and a renderer robot needs no display, so the whole stage is
// built headlessly.
struct stage
{
    stage()
            : loop(inline_workers)
            , scene(threepp::Scene::create())
            , published(std::make_shared<arm_publisher>())
            , shown(two_joint_handle(), attached_models{}, *scene, loop.main_strand(), published->reader(), praxis::rigid_motion::baseline().screw,
                    praxis::rigid_motion::screw_slot_set{})
    {
        published->publish(std::make_shared<const arm_snapshot>(at_rest(configuration(0.0, 0.0))));
        REQUIRE(shown.initialize().has_value());
        REQUIRE(shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
        REQUIRE(shown.set_pose_path("recorded", two_poses()).has_value());
    }

    void draw()
    {
        REQUIRE(loop.main_strand().post([this] { shown.render(); }).has_value());
        REQUIRE(loop.drain().has_value());
        scene->updateMatrixWorld(true);
    }

    threepp::Object3D *figure(std::size_t at)
    {
        return chain_node(*scene, loadable_robot_stencil::solution_figure_name(at));
    }

    threepp::Object3D *segment(std::size_t at, std::size_t segment)
    {
        return chain_part(*scene, loadable_robot_stencil::solution_figure_name(at), loadable_robot_stencil::solution_segment_name(at, segment));
    }

    threepp::Object3D *mark(std::size_t at, std::size_t joint)
    {
        return chain_part(*scene, loadable_robot_stencil::solution_figure_name(at), loadable_robot_stencil::solution_mark_name(at, joint));
    }

    scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<arm_publisher> published;
    loadable_robot_stencil shown;
};

std::vector<joint_vector> two_configurations()
{
    return {configuration(0.2, 0.4), configuration(-0.3, 0.1)};
}

}

TEST_CASE("every combination of the five switches leaves the scene in the state those five name", "[manipulator][solutions]")
{
    stage headless;
    REQUIRE(headless.shown.set_solution_figures(two_configurations()).has_value());

    for(int combination = 0; combination < 32; ++combination)
    {
        const bool meshes  = (combination & 1) != 0;
        const bool axes    = (combination & 2) != 0;
        const bool stick   = (combination & 4) != 0;
        const bool path    = (combination & 8) != 0;
        const bool figures = (combination & 16) != 0;

        headless.shown.set_meshes_shown(meshes);
        headless.shown.set_decoration_shown(axes);
        headless.shown.set_chain_shown(stick);
        REQUIRE(headless.shown.set_pose_path_shown("recorded", path).has_value());
        headless.shown.set_solution_figures_shown(figures);
        headless.draw();

        CHECK(drawn(rendered_arm(*headless.scene)) == meshes);
        CHECK(drawn(first_line_under(*headless.scene, loadable_robot_stencil::joint_axis_name(0))) == axes);
        CHECK(drawn(chain_node(*headless.scene, loadable_robot_stencil::chain_name())) == stick);
        CHECK(drawn(first_line_under(*headless.scene, loadable_robot_stencil::pose_path_name("recorded"))) == path);
        CHECK(drawn(headless.figure(0)) == figures);
    }
}

TEST_CASE("a figure carries one segment per gap between its folded points and one mark per joint, under the counted names", "[manipulator][solutions]")
{
    stage headless;
    REQUIRE(headless.shown.set_solution_figures(two_configurations()).has_value());
    headless.draw();

    for(std::size_t at = 0; at < 2u; ++at)
    {
        CHECK(headless.figure(at) != nullptr);
        for(std::size_t segment = 0; segment < 3u; ++segment)
            CHECK(headless.segment(at, segment) != nullptr);
        for(std::size_t joint = 0; joint < 2u; ++joint)
            CHECK(headless.mark(at, joint) != nullptr);
    }

    CHECK(headless.segment(0, 3) == nullptr);
    CHECK(headless.mark(0, 2) == nullptr);
    CHECK(headless.figure(2) == nullptr);
}

TEST_CASE("a second set of configurations replaces the figures standing and keeps the switch they were hidden by", "[manipulator][solutions]")
{
    stage headless;
    REQUIRE(headless.shown.set_solution_figures(two_configurations()).has_value());
    headless.shown.set_solution_figures_shown(false);

    const std::vector<joint_vector> three{configuration(0.1, 0.1), configuration(0.2, 0.2), configuration(0.3, 0.3)};
    REQUIRE(headless.shown.set_solution_figures(three).has_value());
    headless.draw();

    CHECK(headless.figure(2) != nullptr);
    CHECK_FALSE(drawn(headless.figure(0)));

    headless.shown.set_solution_figures_shown(true);
    headless.draw();

    CHECK(drawn(headless.figure(0)));
}

TEST_CASE("no configuration at all leaves no figure standing and the group still there to be told again", "[manipulator][solutions]")
{
    stage headless;
    REQUIRE(headless.shown.set_solution_figures(two_configurations()).has_value());

    REQUIRE(headless.shown.set_solution_figures(std::vector<joint_vector>{}).has_value());
    headless.draw();

    CHECK(headless.figure(0) == nullptr);

    REQUIRE(headless.shown.set_solution_figures(two_configurations()).has_value());
    headless.draw();

    CHECK(drawn(headless.figure(1)));
}

TEST_CASE("a configuration whose width is not the joint count is declined by name and leaves the figures standing", "[manipulator][solutions]")
{
    stage headless;
    REQUIRE(headless.shown.set_solution_figures(two_configurations()).has_value());
    headless.draw();

    joint_vector too_wide(3);
    too_wide << 0.1, 0.2, 0.3;

    praxis::expected<void, praxis::refusal> answered;
    const std::string reported = praxis::tests::reported_by(
            [&]
            {
                const std::vector<joint_vector> refused{configuration(0.2, 0.4), too_wide};
                answered = headless.shown.set_solution_figures(refused);
            });
    headless.draw();

    CHECK_FALSE(answered.has_value());
    CHECK_THAT(reported, Catch::Matchers::ContainsSubstring("2 joints") && Catch::Matchers::ContainsSubstring("names 3"));
    CHECK(headless.figure(0) != nullptr);
    CHECK(headless.figure(1) != nullptr);
    CHECK(headless.figure(2) == nullptr);
}
