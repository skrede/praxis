#include "fixtures.h"
#include "captured_log.h"

#include "../presets/drawn_lines.h"

#include "praxis/manipulator/arm_snapshot.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/math/Color.hpp>

#include <threepp/materials/interfaces.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <string_view>

using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

// What a point read back out of a float buffer and turned out of the renderer's world is comparable
// at. Every position below is a multiple of an eighth, which single precision carries exactly, so
// the round trip is a comparison rather than a tolerance.
constexpr double read_back = 1.0e-6;

praxis::transform standing_at(double x, double y, double z)
{
    praxis::transform put = praxis::transform::Identity();
    put.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);

    return put;
}

std::vector<praxis::transform> two_poses()
{
    return {standing_at(0.25, 0.5, 0.125), standing_at(-0.75, 0.25, 0.875)};
}

std::vector<praxis::transform> five_poses()
{
    return {standing_at(0.125, 0.0, 0.25), standing_at(0.375, 0.125, 0.5), standing_at(0.625, 0.25, 0.75), standing_at(0.875, 0.375, 1.0), standing_at(1.125, 0.5, 1.25)};
}

std::vector<Eigen::Vector3d> positions_of(const std::vector<praxis::transform> &poses)
{
    std::vector<Eigen::Vector3d> points;
    points.reserve(poses.size());
    for(const praxis::transform &at : poses)
        points.push_back(at.block<3, 1>(0, 3));

    return points;
}

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

praxis::screw_axis revolute_screw(const Eigen::Vector3d &through, const Eigen::Vector3d &along)
{
    return praxis::rigid_motion::baseline().screw.screw_axis_from_point_direction_pitch(through, along, 0.0).value();
}

std::vector<praxis::screw_axis> two_axes()
{
    return {revolute_screw(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ()), revolute_screw(Eigen::Vector3d(static_cast<double>(link_length), 0.0, 0.0), Eigen::Vector3d::UnitZ())};
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
    }

    void draw()
    {
        REQUIRE(loop.main_strand().post([this] { shown.render(); }).has_value());
        REQUIRE(loop.drain().has_value());
        scene->updateMatrixWorld(true);
    }

    std::vector<Eigen::Vector3d> path(std::string_view named)
    {
        return line_in_world(*scene, loadable_robot_stencil::pose_path_name(named));
    }

    threepp::Object3D *path_node(std::string_view named)
    {
        return first_line_under(*scene, loadable_robot_stencil::pose_path_name(named));
    }

    threepp::Object3D *axis_node(std::size_t joint)
    {
        return first_line_under(*scene, loadable_robot_stencil::joint_axis_name(joint));
    }

    threepp::Object3D *chain_node()
    {
        return scene->getObjectByName<threepp::Object3D>(loadable_robot_stencil::chain_name());
    }

    std::size_t lines_under(std::string_view named)
    {
        std::size_t found           = 0;
        threepp::Object3D *carrying = scene->getObjectByName<threepp::Object3D>(loadable_robot_stencil::pose_path_name(named));
        if(carrying == nullptr)
            return found;

        carrying->traverse(
                [&found](threepp::Object3D &at)
                {
                    if(at.type() == "Line")
                        ++found;
                });

        return found;
    }

    scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<arm_publisher> published;
    loadable_robot_stencil shown;
};

bool same_points(const std::vector<Eigen::Vector3d> &read, const std::vector<Eigen::Vector3d> &told)
{
    if(read.size() != told.size())
        return false;

    for(std::size_t at = 0; at < read.size(); ++at)
        if((read[at] - told[at]).norm() > read_back)
            return false;

    return true;
}

}

TEST_CASE("a path told two poses is drawn through their positions, in the space frame and in order", "[manipulator][path]")
{
    stage headless;
    const std::vector<praxis::transform> told = two_poses();

    REQUIRE(headless.shown.set_pose_path("commanded", told).has_value());
    headless.draw();

    const std::vector<Eigen::Vector3d> read = headless.path("commanded");
    REQUIRE(read.size() == 2u);
    CHECK(same_points(read, positions_of(told)));
}

TEST_CASE("a path told five poses draws five points in the order they arrived", "[manipulator][path]")
{
    stage headless;
    const std::vector<praxis::transform> told = five_poses();

    REQUIRE(headless.shown.set_pose_path("traversed", told).has_value());
    headless.draw();

    const std::vector<Eigen::Vector3d> read = headless.path("traversed");
    REQUIRE(read.size() == 5u);
    CHECK(same_points(read, positions_of(told)));
}

TEST_CASE("a path opens at the recorded tone and wears the tone it was told when one is named", "[manipulator][path]")
{
    stage headless;

    REQUIRE(headless.shown.set_pose_path("commanded", two_poses()).has_value());
    REQUIRE(headless.shown.set_pose_path("traversed", two_poses(), threepp::Color::blue).has_value());
    headless.draw();

    const auto *opened = headless.path_node("commanded")->materialAs<threepp::MaterialWithColor>();
    const auto *named  = headless.path_node("traversed")->materialAs<threepp::MaterialWithColor>();
    REQUIRE(opened != nullptr);
    REQUIRE(named != nullptr);

    CHECK(opened->color.equals(threepp::Color(threepp::Color::deepskyblue)));
    CHECK(named->color.equals(threepp::Color(threepp::Color::blue)));
}

TEST_CASE("two named paths both stand, and hiding one by name leaves the other drawn", "[manipulator][path]")
{
    stage headless;
    const std::vector<praxis::transform> first  = two_poses();
    const std::vector<praxis::transform> second = five_poses();

    REQUIRE(headless.shown.set_pose_path("commanded", first).has_value());
    REQUIRE(headless.shown.set_pose_path("traversed", second).has_value());
    headless.draw();

    CHECK(same_points(headless.path("commanded"), positions_of(first)));
    CHECK(same_points(headless.path("traversed"), positions_of(second)));

    REQUIRE(headless.shown.set_pose_path_shown("commanded", false).has_value());
    headless.draw();

    CHECK_FALSE(drawn(headless.path_node("commanded")));
    CHECK(drawn(headless.path_node("traversed")));
}

TEST_CASE("a path told fewer than two poses is declined by name, says so once, and draws nothing", "[manipulator][path]")
{
    stage headless;
    const std::vector<praxis::transform> one{standing_at(0.25, 0.5, 0.125)};
    praxis::expected<void, praxis::refusal> answered{};

    const std::string reported = praxis::tests::reported_by([&] { answered = headless.shown.set_pose_path("commanded", one); });

    REQUIRE_FALSE(answered.has_value());
    CHECK(answered.error() == praxis::refusal::unsupported_input);
    CHECK_THAT(reported, Catch::Matchers::ContainsSubstring("1") && Catch::Matchers::ContainsSubstring("commanded"));

    headless.draw();

    CHECK(headless.path("commanded").empty());
    CHECK(headless.path_node("commanded") == nullptr);
}

TEST_CASE("a path told no poses at all is declined the same way", "[manipulator][path]")
{
    stage headless;
    praxis::expected<void, praxis::refusal> answered{};

    const std::string reported = praxis::tests::reported_by([&] { answered = headless.shown.set_pose_path("commanded", {}); });

    REQUIRE_FALSE(answered.has_value());
    CHECK(answered.error() == praxis::refusal::unsupported_input);
    CHECK_THAT(reported, Catch::Matchers::ContainsSubstring("0"));

    headless.draw();

    CHECK(headless.path_node("commanded") == nullptr);
}

TEST_CASE("a name already drawn carries the second run, and no second line stands under it", "[manipulator][path]")
{
    stage headless;
    const std::vector<praxis::transform> again = five_poses();

    REQUIRE(headless.shown.set_pose_path("commanded", two_poses()).has_value());
    headless.draw();
    REQUIRE(headless.path("commanded").size() == 2u);

    REQUIRE(headless.shown.set_pose_path("commanded", again).has_value());
    headless.draw();

    CHECK(same_points(headless.path("commanded"), positions_of(again)));
    CHECK(headless.lines_under("commanded") == 1u);
}

TEST_CASE("a path hidden by name is still hidden when that name carries a new run, and one never hidden is still drawn", "[manipulator][path]")
{
    stage headless;
    const std::vector<praxis::transform> again = five_poses();

    REQUIRE(headless.shown.set_pose_path("commanded", two_poses()).has_value());
    REQUIRE(headless.shown.set_pose_path("traversed", two_poses()).has_value());
    headless.draw();

    REQUIRE(headless.shown.set_pose_path_shown("commanded", false).has_value());
    headless.draw();

    REQUIRE_FALSE(drawn(headless.path_node("commanded")));
    REQUIRE(drawn(headless.path_node("traversed")));

    REQUIRE(headless.shown.set_pose_path("commanded", again).has_value());
    REQUIRE(headless.shown.set_pose_path("traversed", again).has_value());
    headless.draw();

    CHECK_FALSE(drawn(headless.path_node("commanded")));
    CHECK(same_points(headless.path("commanded"), positions_of(again)));
    CHECK(drawn(headless.path_node("traversed")));
    CHECK(same_points(headless.path("traversed"), positions_of(again)));
}

TEST_CASE("a path hidden through the scene itself is still hidden when its name carries a new run", "[manipulator][path]")
{
    stage headless;
    const std::vector<praxis::transform> again = five_poses();

    REQUIRE(headless.shown.set_pose_path("commanded", two_poses()).has_value());
    headless.draw();
    REQUIRE(drawn(headless.path_node("commanded")));

    threepp::Object3D *standing = headless.path_node("commanded");
    REQUIRE(standing != nullptr);
    standing->visible = false;
    headless.draw();
    REQUIRE_FALSE(drawn(headless.path_node("commanded")));

    REQUIRE(headless.shown.set_pose_path("commanded", again).has_value());
    headless.draw();

    CHECK_FALSE(drawn(headless.path_node("commanded")));
    CHECK(same_points(headless.path("commanded"), positions_of(again)));
}

TEST_CASE("clearing one path by name leaves the other standing", "[manipulator][path]")
{
    stage headless;
    const std::vector<praxis::transform> second = five_poses();

    REQUIRE(headless.shown.set_pose_path("commanded", two_poses()).has_value());
    REQUIRE(headless.shown.set_pose_path("traversed", second).has_value());
    headless.draw();

    headless.shown.clear_pose_path("commanded");
    headless.draw();

    CHECK(headless.path_node("commanded") == nullptr);
    CHECK(same_points(headless.path("traversed"), positions_of(second)));
}

TEST_CASE("clearing every path leaves none, the way the screw axes are cleared", "[manipulator][path]")
{
    stage headless;

    REQUIRE(headless.shown.set_pose_path("commanded", two_poses()).has_value());
    REQUIRE(headless.shown.set_pose_path("traversed", five_poses()).has_value());
    headless.draw();

    headless.shown.clear_pose_paths();
    headless.draw();

    CHECK(headless.path_node("commanded") == nullptr);
    CHECK(headless.path_node("traversed") == nullptr);
}

TEST_CASE("hiding the screw axes and the chain figure leaves every path drawn", "[manipulator][path]")
{
    stage headless;

    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    REQUIRE(headless.shown.set_pose_path("commanded", two_poses()).has_value());
    REQUIRE(headless.shown.set_pose_path("traversed", five_poses()).has_value());
    headless.draw();

    headless.shown.set_decoration_shown(false);
    headless.shown.set_chain_shown(false);
    headless.draw();

    CHECK_FALSE(drawn(headless.axis_node(0)));
    CHECK_FALSE(drawn(headless.chain_node()));
    CHECK(drawn(headless.path_node("commanded")));
    CHECK(drawn(headless.path_node("traversed")));
}

TEST_CASE("hiding a path leaves the arm's meshes, its screw axes and its chain drawn", "[manipulator][path]")
{
    stage headless;

    REQUIRE(headless.shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    REQUIRE(headless.shown.set_pose_path("commanded", two_poses()).has_value());
    headless.draw();

    REQUIRE(headless.shown.set_pose_path_shown("commanded", false).has_value());
    headless.draw();

    CHECK_FALSE(drawn(headless.path_node("commanded")));
    CHECK(drawn(headless.axis_node(0)));
    CHECK(drawn(headless.chain_node()));
    CHECK(headless.shown.robot().visible);
}

TEST_CASE("showing or hiding a name no path stands under is declined and says so", "[manipulator][path]")
{
    stage headless;
    const std::vector<praxis::transform> told = two_poses();
    praxis::expected<void, praxis::refusal> answered{};

    REQUIRE(headless.shown.set_pose_path("commanded", told).has_value());
    headless.draw();

    const std::string reported = praxis::tests::reported_by([&] { answered = headless.shown.set_pose_path_shown("traversed", false); });

    REQUIRE_FALSE(answered.has_value());
    CHECK(answered.error() == praxis::refusal::unsupported_input);
    CHECK_THAT(reported, Catch::Matchers::ContainsSubstring("traversed"));

    headless.draw();

    CHECK(drawn(headless.path_node("commanded")));
    CHECK(same_points(headless.path("commanded"), positions_of(told)));

    CHECK(headless.shown.set_pose_path_shown("commanded", false).has_value());
}
