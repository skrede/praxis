#include "window_stage.h"
#include "captured_log.h"

#include "../presets/drawn_lines.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/motion_drawings.h"
#include "praxis/manipulator/path_comparison_window.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/angles.h"
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

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;
using Catch::Matchers::ContainsSubstring;

namespace {

constexpr const char *panel_title   = "Comparison";
constexpr const char *comparison_at = "machine/path_comparison";

// What a point read back out of a float buffer and turned out of the renderer's world is comparable
// at, and the tolerance the two ends of the joint-space run are held to against the forward map.
constexpr double read_back  = 1.0e-5;
constexpr double at_the_end = 1.0e-9;

// How far above the last control each of these stands, which is what a keyboard walk counts. The
// cases below read `state()` back to say which of them moved rather than trusting the count.
constexpr std::size_t screw_switch       = 2;
constexpr std::size_t joint_space_switch = 4;
constexpr std::size_t first_end          = 6;

// How far apart the poses of a published traversed run stand, in metres along one axis.
constexpr double traversed_step = 0.05;

const Eigen::Vector3d origin(Eigen::Vector3d::Zero());
const rotation upright(rotation::Identity());

screw_axis revolute_screw(const Eigen::Vector3d &through, const Eigen::Vector3d &along)
{
    return rigid_motion::baseline().screw.screw_axis_from_point_direction_pitch(through, along, 0.0).value();
}

// A planar two-revolute chain against the width the two-joint handle reports, so the three shapes
// over a pair of its configurations are three genuinely different curves.
screw_chain planar_chain()
{
    joint_limits bounds{};
    bounds.velocity       = joint_vector::Constant(2, 1.0);
    bounds.acceleration   = joint_vector::Constant(2, 4.0);
    bounds.lower_position = joint_vector::Constant(2, -3.0);
    bounds.upper_position = joint_vector::Constant(2, 3.0);

    transform home = transform::Identity();
    home(0, 3)     = 2.0 * static_cast<double>(link_length);

    return screw_chain(home, {revolute_screw(origin, Eigen::Vector3d::UnitZ()), revolute_screw(Eigen::Vector3d(link_length, 0.0, 0.0), Eigen::Vector3d::UnitZ())}, bounds);
}

transform pose_at(const joint_vector &joints)
{
    const screw_chain chain = planar_chain();

    return baseline().fk.forward_kinematics(chain.home, chain.space_screws, joints).value();
}

// A run of poses along the space frame's first axis, which is a drawing of at least two poses and
// nothing the three commanded shapes pass through.
std::vector<transform> traversed_run(std::size_t poses)
{
    std::vector<transform> through;
    for(std::size_t step = 0; step < poses; ++step)
    {
        transform at = transform::Identity();
        at(0, 3)     = static_cast<double>(step) * traversed_step;
        through.push_back(at);
    }

    return through;
}

path_comparison_window::settings ends_at(const joint_vector &first, const joint_vector &second)
{
    return path_comparison_window::settings{first, second};
}

struct stage
{
    explicit stage(const path_comparison_window::settings &opened = ends_at(configuration(0.0, 0.0), configuration(1.0, 1.0)))
            : loop(inline_workers)
            , scene(threepp::Scene::create())
            , published(publishing(at_rest(configuration(0.0, 0.0), origin, upright)))
            , stencil(two_joint_handle(), attached_models{}, *scene, loop.main_strand(), published->reader(), rigid_motion::baseline().screw, rigid_motion::screw_slot_set{})
            , panel(panel_title, published->reader(), std::weak_ptr<owned_arm>{}, stencil, baseline().fk, planar_chain(), trajectory::baseline().path, opened, comparison_at)
    {
        REQUIRE(stencil.initialize().has_value());
    }

    void draw()
    {
        frames.draw(over(panel));
        scene->updateMatrixWorld(true);
    }

    std::vector<Eigen::Vector3d> path(std::string_view named)
    {
        return line_in_world(*scene, loadable_robot_stencil::pose_path_name(named));
    }

    bool shown(std::string_view named)
    {
        return drawn(first_line_under(*scene, loadable_robot_stencil::pose_path_name(named)));
    }

    threepp::Object3D *path_node(std::string_view named)
    {
        return first_line_under(*scene, loadable_robot_stencil::pose_path_name(named));
    }

    void publish_traversed(const std::vector<transform> &through)
    {
        arm_snapshot seen = at_rest(configuration(0.0, 0.0), origin, upright);
        seen.traversed    = std::make_shared<const std::vector<transform>>(through);
        published->publish(std::make_shared<const arm_snapshot>(seen));
    }

    // Types into the leftmost component of the end standing that many controls above the last one.
    void type_into_end(std::size_t steps, const char *degrees_text)
    {
        const drawing draw = over(panel);
        start_navigating(frames, draw);
        reach(frames, draw, ImGuiKey_End);
        for(std::size_t step = 0; step < steps; ++step)
            tap(frames, draw, ImGuiKey_UpArrow);

        tap(frames, draw, ImGuiKey_Space);
        for(const char *at = degrees_text; *at != '\0'; ++at)
            ImGui::GetIO().AddInputCharacter(static_cast<unsigned int>(*at));
        frames.draw_frame(draw);
        tap(frames, draw, ImGuiKey_Enter);
        scene->updateMatrixWorld(true);
    }

    // Stands the keyboard cursor a stated number of controls above the last one and presses it.
    void press_above_last(std::size_t steps)
    {
        const drawing draw = over(panel);
        start_navigating(frames, draw);
        reach(frames, draw, ImGuiKey_End);
        for(std::size_t step = 0; step < steps; ++step)
            tap(frames, draw, ImGuiKey_UpArrow);
        tap(frames, draw, ImGuiKey_Space);
        scene->updateMatrixWorld(true);
    }

    imgui_frame frames;
    praxis::scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<arm_publisher> published;
    loadable_robot_stencil stencil;
    path_comparison_window panel;
};

}

TEST_CASE("the three shapes over one pair of ends stand under three names at the count they share", "[manipulator][comparison]")
{
    stage over;
    over.draw();

    for(const char *named : {path_comparison_window::joint_space_path, path_comparison_window::decoupled_path, path_comparison_window::screw_path})
    {
        INFO(named);
        CHECK(over.path(named).size() == path_comparison_window::drawn_points);
    }
}

TEST_CASE("a published traversed run stands beside the three shapes, under its own name and tone", "[manipulator][comparison]")
{
    stage over;
    over.draw();

    CHECK(over.path_node(traversed_motion_path) == nullptr);

    over.publish_traversed(traversed_run(path_comparison_window::drawn_points));
    over.draw();

    REQUIRE(over.path(traversed_motion_path).size() == path_comparison_window::drawn_points);

    const auto *tone = over.path_node(traversed_motion_path)->materialAs<threepp::MaterialWithColor>();

    REQUIRE(tone != nullptr);
    CHECK(tone->color.equals(threepp::Color(traversed_motion_tone)));
}

// The traversed run is told on its own occasion rather than behind the guard that holds the three
// commanded shapes still, so moving an end reaches the three and leaves the fourth where it was.
TEST_CASE("changing an end rebuilds the three shapes and leaves the traversed polyline standing", "[manipulator][comparison]")
{
    stage over;
    over.draw();
    over.publish_traversed(traversed_run(path_comparison_window::drawn_points));
    over.draw();

    const std::vector<Eigen::Vector3d> traversed = over.path(traversed_motion_path);
    const threepp::Object3D *standing            = over.path_node(traversed_motion_path);
    const threepp::Object3D *commanded           = over.path_node(path_comparison_window::joint_space_path);

    REQUIRE(traversed.size() == path_comparison_window::drawn_points);
    REQUIRE(commanded != nullptr);

    over.type_into_end(first_end, "35.5");
    over.draw();

    CHECK(over.path_node(path_comparison_window::joint_space_path) != commanded);
    CHECK(over.path_node(traversed_motion_path) == standing);
    CHECK(over.path(traversed_motion_path) == traversed);
}

// Three polylines sampled at different parameters are not comparable point for point, so what is
// asserted is that the three carry the same number of points and that the joint-space one begins and
// ends at the forward map of the two ends rather than somewhere along the way.
TEST_CASE("the three shapes are sampled at the same path parameters", "[manipulator][comparison]")
{
    stage over;
    over.draw();

    const std::vector<Eigen::Vector3d> joint_space = over.path(path_comparison_window::joint_space_path);
    const std::vector<Eigen::Vector3d> decoupled   = over.path(path_comparison_window::decoupled_path);
    const std::vector<Eigen::Vector3d> screw       = over.path(path_comparison_window::screw_path);

    REQUIRE(joint_space.size() == decoupled.size());
    REQUIRE(joint_space.size() == screw.size());

    // Against the ends the window holds rather than the ones it was handed: the ends are edited in
    // degrees and held in single precision, so a radian value written into a settings struct comes
    // back one float round trip away from where it started.
    const std::vector<transform> mapped = over.panel.poses_along(compared_path::joint_space);

    REQUIRE(mapped.size() == path_comparison_window::drawn_points);
    CHECK(mapped.front().isApprox(pose_at(over.panel.state().first), at_the_end));
    CHECK(mapped.back().isApprox(pose_at(over.panel.state().second), at_the_end));
    CHECK((joint_space.front() - mapped.front().block<3, 1>(0, 3)).norm() < read_back);
    CHECK((joint_space.back() - mapped.back().block<3, 1>(0, 3)).norm() < read_back);
}

// The two task-space shapes join the same pair of poses and take different routes between them, so
// what separates them in the picture is the shape each draws.
TEST_CASE("the screw shape bows away from the straight line the decoupled shape draws", "[manipulator][comparison]")
{
    stage over;
    over.draw();

    CHECK(apart(over.path(path_comparison_window::screw_path), over.path(path_comparison_window::decoupled_path)) > 0.05);
    CHECK(apart(over.path(path_comparison_window::joint_space_path), over.path(path_comparison_window::decoupled_path)) > 0.05);
}

TEST_CASE("hiding one shape leaves the other two drawn and shown", "[manipulator][comparison]")
{
    stage over;
    over.draw();

    REQUIRE(over.shown(path_comparison_window::screw_path));

    over.press_above_last(screw_switch);

    REQUIRE_FALSE(over.panel.state().screw);
    CHECK(over.panel.state().joint_space);
    CHECK(over.panel.state().decoupled);
    CHECK_FALSE(over.shown(path_comparison_window::screw_path));
    CHECK(over.shown(path_comparison_window::joint_space_path));
    CHECK(over.shown(path_comparison_window::decoupled_path));
}

TEST_CASE("hiding the joint-space shape reaches neither of the two beside it", "[manipulator][comparison]")
{
    stage over;
    over.draw();

    over.press_above_last(joint_space_switch);

    REQUIRE_FALSE(over.panel.state().joint_space);
    CHECK_FALSE(over.shown(path_comparison_window::joint_space_path));
    CHECK(over.shown(path_comparison_window::decoupled_path));
    CHECK(over.shown(path_comparison_window::screw_path));
}

TEST_CASE("an end of the wrong width is declined by name and the end beside it still stands", "[manipulator][comparison]")
{
    captured_log log;

    stage over(ends_at(joint_vector::Constant(5, 0.25), configuration(1.0, 1.0)));

    CHECK_THAT(log.text(), ContainsSubstring("first end of 5 joint values for an arm of 2 joints"));
    CHECK(over.panel.state().first.size() == 2);
    CHECK(over.panel.state().second.isApprox(configuration(1.0, 1.0), read_back));
}

TEST_CASE("a comparison the settings name no end for opens at the pair its own opening answers", "[manipulator][comparison]")
{
    stage over(path_comparison_window::settings{});

    CHECK(over.panel.state().first.isApprox(path_comparison_window::opening_first(2), read_back));
    CHECK(over.panel.state().second.isApprox(path_comparison_window::opening_second(2), read_back));
}

TEST_CASE("the chosen shape is the one the settings named", "[manipulator][comparison]")
{
    stage over(path_comparison_window::settings{configuration(0.0, 0.0), configuration(1.0, 1.0), true, true, true, compared_path::decoupled});

    CHECK(over.panel.state().played == compared_path::decoupled);
}
