#include "window_stage.h"
#include "captured_log.h"

#include "../presets/drawn_lines.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/motion_drawings.h"
#include "praxis/manipulator/path_comparison_window.h"
#include "praxis/manipulator/baseline/robot.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/math/Color.hpp>

#include <threepp/materials/interfaces.hpp>

#include <Eigen/Core>

#include <span>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <algorithm>
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
constexpr std::size_t play_button        = 0;

// How far apart the poses of a published traversed run stand, in metres along one axis.
constexpr double traversed_step = 0.05;

// The three commanded drawings in one order: the name each stands under on the stencil and the shape
// each is sampled from, read by the same index.
constexpr std::array<const char *, 3> drawn_names{path_comparison_window::joint_space_path, path_comparison_window::decoupled_path, path_comparison_window::screw_path};
constexpr std::array<compared_path, 3> drawn_shapes{compared_path::joint_space, compared_path::decoupled, compared_path::screw};

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

    return baseline().fk.forward_kinematics(rigid_motion::baseline().screw, chain.home, chain.space_screws, joints).value();
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

std::vector<transform> &handed_to_the_factory()
{
    static std::vector<transform> handed;

    return handed;
}

std::vector<transform> &carried_from_the_tool_frame()
{
    static std::vector<transform> carried;

    return carried;
}

expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> recorded_waypoints(const kinematics &, std::span<const transform> waypoints, const joint_vector &,
                                                                                        const joint_limits &)
{
    handed_to_the_factory().assign(waypoints.begin(), waypoints.end());

    return unexpected(refusal::not_implemented);
}

transform recorded_conversion(const rigid_motion::frame_ops &frames, const transform &tool_pose, const transform &tool_offset)
{
    carried_from_the_tool_frame().push_back(tool_pose);

    return flange_pose_from_tool_pose(frames, tool_pose, tool_offset);
}

task_trajectory_ops recording_waypoints()
{
    return task_trajectory_ops{&recorded_waypoints};
}

robot_ops recording_the_conversion()
{
    robot_ops injected                  = baseline().robot;
    injected.flange_pose_from_tool_pose = &recorded_conversion;

    return injected;
}

transform bent_tool_offset()
{
    const rigid_motion::frame_ops frames = rigid_motion::baseline().frame;

    return frames.transformation_matrix_from_rotation_position(frames.rotate_z(0.35), Eigen::Vector3d(0.132, 0.082, 0.0));
}

// The publication a stage opens on where a tool stands at the flange. Every other member is what an
// arm at rest reports, so the offset is the one thing separating this from the opening publication.
arm_snapshot tooled_with(const transform &offset)
{
    arm_snapshot seen = at_rest(configuration(0.0, 0.0), origin, upright);
    seen.tool_offset  = offset;

    return seen;
}

std::vector<Eigen::Vector3d> positions_of(const std::vector<transform> &poses)
{
    std::vector<Eigen::Vector3d> standing;
    for(const transform &pose : poses)
        standing.push_back(pose.block<3, 1>(0, 3));

    return standing;
}

std::vector<transform> carried_to_the_tool_frame(const std::vector<transform> &flange_poses)
{
    std::vector<transform> standing;
    for(const transform &pose : flange_poses)
        standing.push_back(tool_pose_from_flange_pose(pose, bent_tool_offset()));

    return standing;
}

double reach_of(const transform &offset)
{
    return offset.block<3, 1>(0, 3).norm();
}

// The greatest a read line's separation from a run of positions departs from one stated distance. A
// pose and the pose a tool offset carries it to stand that offset's own translation length apart at
// every sample whatever the rotation there, so a converted drawing answers zero.
double off_the_reach(const std::vector<Eigen::Vector3d> &drawn, const std::vector<Eigen::Vector3d> &from, double reach)
{
    double worst = 0.0;
    for(std::size_t at = 0; at < drawn.size() && at < from.size(); ++at)
        worst = std::max(worst, std::abs((drawn[at] - from[at]).norm() - reach));

    return worst;
}

double worst_gap(const std::vector<transform> &one, const std::vector<transform> &other)
{
    double worst = 0.0;
    for(std::size_t step = 0; step < one.size(); ++step)
        worst = std::max(worst, (one[step] - other[step]).cwiseAbs().maxCoeff());

    return worst;
}

struct stage
{
    explicit stage(const path_comparison_window::settings &opened = ends_at(configuration(0.0, 0.0), configuration(1.0, 1.0)), std::weak_ptr<owned_arm> driving = {},
                   const arm_snapshot &opening = tooled_with(transform::Identity()))
            : loop(inline_workers)
            , scene(threepp::Scene::create())
            , seen(opening)
            , published(publishing(seen))
            , stencil(two_joint_handle(), attachments{}, *scene, loop.main_strand(), published->reader(), rigid_motion::baseline().screw, rigid_motion::screw_slot_set{})
            , panel(panel_title, published->reader(), std::move(driving), stencil, rigid_motion::baseline().screw, baseline().fk, planar_chain(), trajectory::baseline().path,
                    baseline().robot, opened, comparison_at)
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

    std::array<threepp::Object3D *, 3> commanded_nodes()
    {
        std::array<threepp::Object3D *, 3> standing{};
        for(std::size_t at = 0; at < drawn_names.size(); ++at)
            standing[at] = path_node(drawn_names[at]);

        return standing;
    }

    void publish_traversed(const std::vector<transform> &through)
    {
        seen.traversed = std::make_shared<const std::vector<transform>>(through);
        published->publish(std::make_shared<const arm_snapshot>(seen));
    }

    void publish_tool_offset(const transform &offset)
    {
        seen.tool_offset = offset;
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
    arm_snapshot seen;
    std::shared_ptr<arm_publisher> published;
    loadable_robot_stencil stencil;
    path_comparison_window panel;
};

struct driven_by_the_window
{
    driven_by_the_window()
            : loop(inline_workers)
            , driven(compose(loop, composing_motion(), rigid_motion::baseline().screw, reference_framing(), recording_waypoints(), recording_the_conversion()))
    {
        command(std::weak_ptr<owned_arm>(driven.owned), [](robot_controller &, scene_robot &arm) { arm.set_tool_offset(bent_tool_offset()); });
        REQUIRE(loop.drain().has_value());
    }

    praxis::scheduler::scheduler loop;
    composed_arm driven;
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

// A drawing standing the bent tool's own reach from the flange poses the window answers is a drawing
// something converted; one agreeing with the bound conversion of those same poses is a drawing that
// conversion made.
TEST_CASE("the three commanded shapes are drawn at the tool centre point the published offset names", "[manipulator][comparison]")
{
    stage over(ends_at(configuration(0.0, 0.0), configuration(1.0, 1.0)), {}, tooled_with(bent_tool_offset()));
    over.draw();

    for(std::size_t at = 0; at < drawn_names.size(); ++at)
    {
        INFO(drawn_names[at]);

        const std::vector<transform> flange     = over.panel.poses_along(drawn_shapes[at]);
        const std::vector<Eigen::Vector3d> line = over.path(drawn_names[at]);

        REQUIRE(flange.size() == path_comparison_window::drawn_points);
        REQUIRE(line.size() == path_comparison_window::drawn_points);
        CHECK(off_the_reach(line, positions_of(flange), reach_of(bent_tool_offset())) < read_back);
        CHECK(apart(line, positions_of(carried_to_the_tool_frame(flange))) < read_back);
    }
}

// The drawing is latched on the ends and the offset it was last drawn with, so an offset arriving
// after the first frame reaches all three and a frame where neither moved reaches none of them.
TEST_CASE("an offset arriving after the first drawing rebuilds the three shapes and the latch still holds", "[manipulator][comparison]")
{
    stage over;
    over.draw();

    const std::array<threepp::Object3D *, 3> opened = over.commanded_nodes();

    over.publish_tool_offset(bent_tool_offset());
    over.draw();

    const std::array<threepp::Object3D *, 3> rebuilt = over.commanded_nodes();

    over.draw();

    const std::array<threepp::Object3D *, 3> standing = over.commanded_nodes();

    for(std::size_t at = 0; at < drawn_names.size(); ++at)
    {
        INFO(drawn_names[at]);
        REQUIRE(opened[at] != nullptr);
        CHECK(rebuilt[at] != opened[at]);
        CHECK(standing[at] == rebuilt[at]);
        CHECK(apart(over.path(drawn_names[at]), positions_of(carried_to_the_tool_frame(over.panel.poses_along(drawn_shapes[at])))) < read_back);
    }
}

// The two poses the factory is handed at the ends are named through the forward map rather than
// through `poses_along`, so a conversion moved into that accessor or applied a second time on the way
// to the controller fails here while the drawing beside it stands the offset away.
TEST_CASE("playing a task-space shape hands the controller flange poses and draws at the tool centre point", "[manipulator][comparison]")
{
    captured_log log;
    driven_by_the_window arm;
    stage over(path_comparison_window::settings{configuration(0.0, 0.0), configuration(1.0, 1.0), true, true, true, compared_path::screw}, arm.driven.owned,
               tooled_with(bent_tool_offset()));
    over.draw();

    handed_to_the_factory().clear();
    carried_from_the_tool_frame().clear();
    over.press_above_last(play_button);
    REQUIRE(arm.loop.drain().has_value());

    const std::vector<transform> drawn = over.panel.poses_along(compared_path::screw);

    REQUIRE(drawn.size() == path_comparison_window::drawn_points);
    REQUIRE(handed_to_the_factory().size() == drawn.size());
    CHECK(handed_to_the_factory().front().isApprox(pose_at(over.panel.state().first), at_the_end));
    CHECK(handed_to_the_factory().back().isApprox(pose_at(over.panel.state().second), at_the_end));
    CHECK(worst_gap(handed_to_the_factory(), drawn) < read_back);
    REQUIRE(carried_from_the_tool_frame().size() == drawn.size());
    CHECK(worst_gap(carried_from_the_tool_frame(), carried_to_the_tool_frame(drawn)) < read_back);
    CHECK(off_the_reach(over.path(path_comparison_window::screw_path), positions_of(drawn), reach_of(bent_tool_offset())) < read_back);
}
