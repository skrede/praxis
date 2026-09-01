#include "opened_arm.h"
#include "drawn_chain.h"
#include "carried_models.h"
#include "supplied_chain.h"
#include "composed_panels.h"

#include "praxis/presets/arm.h"
#include "praxis/presets/screw_table.h"

#include "praxis/manipulator/screw_chain.h"
#include "praxis/manipulator/control_mode.h"
#include "praxis/manipulator/joint_control_window.h"
#include "praxis/manipulator/screw_modeling_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/preset.h"

#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/configurable.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <threepp/math/Box3.hpp>
#include <threepp/math/Vector3.hpp>
#include <threepp/math/Quaternion.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <filesystem>

using namespace praxis;
using namespace praxis::fixture;

namespace {

const std::vector<std::string> &forward_windows()
{
    static const std::vector<std::string> shown{"Joint control", "Pose", "View"};

    return shown;
}

const std::vector<std::string> &modeling_windows()
{
    static const std::vector<std::string> shown{"Joint control", "Chain", "View"};

    return shown;
}

std::vector<Eigen::Vector3d> axis_of(threepp::Scene &target, std::size_t joint)
{
    const std::vector<Eigen::Vector3d> drawn = line_in_world(target, manipulator::loadable_robot_stencil::joint_axis_name(joint));
    REQUIRE(drawn.size() == 2u);

    return drawn;
}

threepp::Object3D *first_drawn_axis(threepp::Scene &target)
{
    return first_line_under(target, manipulator::loadable_robot_stencil::joint_axis_name(0));
}

// What a composition drew, reached through the stencil it composed rather than through the scene:
// a model nothing installed is absent from both, and only the stencil says which of the two it is.
manipulator::loadable_robot_stencil &drawn_by(const std::shared_ptr<scene::preset> &composed)
{
    REQUIRE(composed != nullptr);

    const auto held = std::dynamic_pointer_cast<manipulator::loadable_robot_stencil>(composed->stencil);
    REQUIRE(held != nullptr);

    return *held;
}

std::shared_ptr<threepp::Object3D> frame_marker_of(const std::shared_ptr<scene::preset> &composed)
{
    return drawn_by(composed).attached_at(manipulator::flange_attachment::frame_marker);
}

// How far the marker reaches, which is the proportion the one builder took of the arm it was built
// over: two scenarios showing markers of different size on one machine separate here.
double marker_extent(threepp::Object3D &marker)
{
    threepp::Box3 around;
    around.setFromObject(marker);

    return static_cast<double>(around.getSize().length());
}

// The points the supplied chain is drawn through. Its last is the place the typed home pose names
// once the published configuration has been walked, which is the chain's own answer for the flange.
std::vector<Eigen::Vector3d> supplied_chain_points(threepp::Scene &target)
{
    return chain_in_world(target, manipulator::loadable_robot_stencil::chain_name(), &manipulator::loadable_robot_stencil::chain_segment_name);
}

// The last joint's field stands below the two option cycles and the five joints before it, and the
// scenario is opened in the mode where a field that moved drives the arm rather than arming a move.
constexpr std::size_t last_joint_field    = 7;
constexpr const char *last_joint_degrees  = "30";
constexpr double turned_by_the_last_joint = 0.25;

presets::arm_scenario driving_on_edit(const std::filesystem::path &description)
{
    presets::arm_scenario chosen = described_by(description);
    chosen.joint_control         = manipulator::joint_control_window::settings(manipulator::control_mode::preview);

    return chosen;
}

}

TEST_CASE("the forward-kinematics scenario composes the joint control, the pose and the view", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_arm built;
    REQUIRE(composed_windows(built.open(chosen, presets::arm_windows_forward(chosen))) == forward_windows());
}

TEST_CASE("every window the forward-kinematics scenario composes opens exactly one panel under its own title", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_arm built;
    each_window_opens_one_panel(built.open(chosen, presets::arm_windows_forward(chosen)));
}

// The two shipped machines are both six-axis arms, so that they both compose says nothing about
// whether the width is a parameter. Two descriptions of different widths through one composer does.
TEST_CASE("one composer serves descriptions of different joint counts", "[presets][windows]")
{
    const described_arm six(6, "six");
    const described_arm three(3, "three");

    const presets::arm_scenario wider  = described_by(six.where);
    const presets::arm_scenario narrow = described_by(three.where);

    opened_arm over_six;
    REQUIRE(composed_windows(over_six.open(wider, presets::arm_windows_forward(wider))) == forward_windows());
    REQUIRE(drawn_axes(*over_six.scene) == 6u);

    opened_arm over_three;
    REQUIRE(composed_windows(over_three.open(narrow, presets::arm_windows_forward(narrow))) == forward_windows());
    REQUIRE(drawn_axes(*over_three.scene) == 3u);
}

TEST_CASE("every drawn axis lies on the axis the derived chain names for that joint", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen     = described_by(described.where);
    const manipulator::screw_chain derived = derived_chain(described.where);

    opened_arm built;
    const std::shared_ptr<scene::preset> composed = built.open(chosen, presets::arm_windows_forward(chosen));
    built.draw(*composed);

    REQUIRE(derived.joint_count() == 6u);
    for(std::size_t joint = 0; joint < derived.joint_count(); ++joint)
    {
        INFO("joint " << joint);

        const std::vector<Eigen::Vector3d> drawn = line_in_world(*built.scene, manipulator::loadable_robot_stencil::joint_axis_name(joint));
        REQUIRE(drawn.size() == 2u);
        REQUIRE(on_axis(derived.space_screws[joint], drawn.front()));
        REQUIRE(on_axis(derived.space_screws[joint], drawn.back()));
    }
}

TEST_CASE("a description that does not load composes no scenario and leaves the scene alone", "[presets][windows]")
{
    const described_arm described(1, "absent");
    const presets::arm_scenario chosen = described_by(described.directory / "no_such_arm.urdf");

    opened_arm built;
    const std::size_t before = built.descendants();

    REQUIRE(built.open(chosen, presets::arm_windows_forward(chosen)) == nullptr);
    REQUIRE(built.descendants() == before);
}

TEST_CASE("the forward-kinematics scenario leaves the scene as it found it", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_arm built;
    const std::size_t before = built.descendants();

    std::shared_ptr<scene::preset> composed = built.open(chosen, presets::arm_windows_forward(chosen));
    REQUIRE(built.descendants() > before);

    built.release(std::move(composed));
    REQUIRE(built.descendants() == before);
}

// The scenario composes no tool window, so nothing running in it can put anything at the flange. A
// frame there is one the composition asked for, and it is the only thing standing beside the arm.
TEST_CASE("the forward-kinematics scenario carries a frame marker at the flange and neither model", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_arm built;
    manipulator::loadable_robot_stencil &drawn = drawn_by(built.open(chosen, presets::arm_windows_forward(chosen)));

    REQUIRE(drawn.attached_at(manipulator::flange_attachment::frame_marker) != nullptr);
    CHECK(drawn.attached_at(manipulator::flange_attachment::tool) == nullptr);
    CHECK(drawn.world_object() == nullptr);
}

// The cue a reader driving the arm was left without: with the tool model hidden the last joint had
// nothing visible to turn. This scenario composes no tool window at all, so a frame that turns here
// is one no window can take away.
TEST_CASE("driving the last joint turns the frame marker the forward-kinematics scenario carries", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = driving_on_edit(described.where);

    opened_arm built;
    const std::shared_ptr<scene::preset> composed = built.open(chosen, presets::arm_windows_forward(chosen));
    const std::shared_ptr<threepp::Object3D> put  = frame_marker_of(composed);
    REQUIRE(put != nullptr);

    built.draw(*composed);
    const threepp::Quaternion before  = put->quaternion;
    const Eigen::Vector3d standing_at = mark_in_world(*put);

    type_at(*composed->windows.front(), last_joint_field, last_joint_degrees);
    REQUIRE(built.loop.drain().has_value());
    built.draw(*composed);

    // The flange stands on the last joint's own axis, so driving it turns the frame and carries
    // nothing on screen anywhere: with no frame drawn there, that joint has no visible cue at all.
    CHECK(static_cast<double>(put->quaternion.angleTo(before)) > turned_by_the_last_joint);
    CHECK((mark_in_world(*put) - standing_at).norm() < read_back);
}

// One published builder over one machine, so a reader moving between the two scenarios that show no
// tool window sees the same frame rather than two markers drawn to different proportions.
TEST_CASE("the two scenarios composing no tool window carry markers of the same proportions", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_arm ahead;
    const std::shared_ptr<threepp::Object3D> forward = frame_marker_of(ahead.open(chosen, presets::arm_windows_forward(chosen)));

    opened_arm beside;
    const std::shared_ptr<threepp::Object3D> modeled = frame_marker_of(beside.open(chosen, presets::arm_windows_modeling(chosen, presets::screw_table_source{})));

    REQUIRE(forward != nullptr);
    REQUIRE(modeled != nullptr);
    CHECK(std::abs(marker_extent(*forward) - marker_extent(*modeled)) < read_back);
}

// The agnosticism case above is over two descriptions written for it. This one is over the two
// machines the demonstration actually offers, each carrying six axes and its own meshes.
TEST_CASE("both deployed machines open the forward-kinematics scenario", "[presets][windows]")
{
    for(const auto &named : {std::pair<const char *, bool>{"ur_description/urdf/ur.urdf.xacro", true}, std::pair<const char *, bool>{"kuka_kr6_support/urdf/kr6r900sixx.xacro", false}})
    {
        INFO(named.first);

        const std::optional<presets::arm_scenario> chosen = deployed_machine(named.first, named.second);
        if(!chosen)
            SKIP("no robot description is deployed for this configuration");

        opened_arm built;
        REQUIRE(composed_windows(built.open(*chosen, presets::arm_windows_forward(*chosen))) == forward_windows());
        REQUIRE(drawn_axes(*built.scene) == 6u);
    }
}

TEST_CASE("the supplied-chain scenario composes the joint control, the chain and the view", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_arm built;
    REQUIRE(composed_windows(built.open(chosen, presets::arm_windows_modeling(chosen, presets::screw_table_source{}))) == modeling_windows());
}

TEST_CASE("every window the supplied-chain scenario composes opens exactly one panel under its own title", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_arm built;
    each_window_opens_one_panel(built.open(chosen, presets::arm_windows_modeling(chosen, presets::screw_table_source{})));
}

// The opening state is the one nobody has modelled, and it is asserted in the scene rather than in a
// struct: every joint's line is the same line through the origin, which no derived chain gives.
TEST_CASE("a scenario keeping no chain opens with every drawn axis coincident at the origin", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_arm built;
    const std::shared_ptr<scene::preset> composed = built.open(chosen, presets::arm_windows_modeling(chosen, presets::screw_table_source{}));
    built.draw(*composed);

    REQUIRE(drawn_axes(*built.scene) == 6u);

    const std::vector<Eigen::Vector3d> first = axis_of(*built.scene, 0);
    REQUIRE((first.front() + first.back()).norm() < read_back);
    for(std::size_t joint = 0; joint < 6u; ++joint)
    {
        INFO("joint " << joint);
        REQUIRE(same_line(axis_of(*built.scene, joint), first));
    }
}

TEST_CASE("a scenario opened at a kept chain draws the axes that chain names", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen     = described_by(described.where);
    const std::vector<screw_axis> supplied = a_supplied_chain(6);

    opened_arm built;
    const std::shared_ptr<scene::preset> composed =
            built.open(chosen, presets::arm_windows_modeling(chosen, supplied_from(kept_chain(supplied, "drawn.xml"), chain_binding("drawn-into.xml"))));
    built.draw(*composed);

    REQUIRE(drawn_axes(*built.scene) == 6u);
    for(std::size_t joint = 0; joint < supplied.size(); ++joint)
    {
        INFO("joint " << joint);

        const std::vector<Eigen::Vector3d> drawn = axis_of(*built.scene, joint);
        REQUIRE(on_axis(supplied[joint], drawn.front()));
        REQUIRE(on_axis(supplied[joint], drawn.back()));
    }
}

TEST_CASE("the supplied-chain scenario hides and restores the rendered arm and can take the drawn axes away nowhere", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_arm built;
    const std::shared_ptr<scene::preset> composed = built.open(chosen, presets::arm_windows_modeling(chosen, presets::screw_table_source{}));
    built.draw(*composed);

    scene::imgui_window &view = *composed->windows.back();
    REQUIRE(drawn(rendered_arm(*built.scene)));

    take_entry_at(view, 0, 1);
    REQUIRE_FALSE(drawn(rendered_arm(*built.scene)));
    take_entry_at(view, 0, 0);
    REQUIRE(drawn(rendered_arm(*built.scene)));

    for(std::size_t entry = 0; entry < 3u; ++entry)
    {
        INFO("entry " << entry);
        take_entry_at(view, 0, entry);
        REQUIRE(drawn(first_drawn_axis(*built.scene)));
    }

    press_at(view, 1);
    REQUIRE(drawn(first_drawn_axis(*built.scene)));
}

// A machine document is free to say the axes open hidden, and every other scenario honors it. This
// one cannot: with no control to bring them back, honoring it would open a scenario whose whole
// subject is off screen and unreachable.
TEST_CASE("a machine asking for the drawn axes hidden still opens the supplied-chain scenario with them on", "[presets][windows]")
{
    const described_arm described(6, "six");
    presets::arm_scenario chosen = described_by(described.where);
    chosen.robot_view.decoration = false;

    opened_arm built;
    const std::shared_ptr<scene::preset> composed = built.open(chosen, presets::arm_windows_modeling(chosen, presets::screw_table_source{}));
    built.draw(*composed);

    REQUIRE(drawn(first_drawn_axis(*built.scene)));
}

TEST_CASE("the supplied-chain scenario leaves the scene as it found it", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_arm built;
    const std::size_t before = built.descendants();

    std::shared_ptr<scene::preset> composed = built.open(chosen, presets::arm_windows_modeling(chosen, presets::screw_table_source{}));
    REQUIRE(built.descendants() > before);

    built.release(std::move(composed));
    REQUIRE(built.descendants() == before);
}

// The window mints the edits and the preset declares the keys they are written under, in two
// modules that never see each other's spelling; a name they disagree on is written nowhere and
// falls back to the degenerate chain, which no assertion on the window's own state would catch.
TEST_CASE("the edits the chain window mints reach the leaves the table's keyspace declares", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen     = described_by(described.where);
    const std::vector<screw_axis> supplied = a_supplied_chain(6);
    const config::binding into             = chain_binding("window-edits.xml");

    opened_arm built;
    const std::shared_ptr<scene::preset> composed = built.open(chosen, presets::arm_windows_modeling(chosen, supplied_from(kept_chain(supplied, "window-edits-source.xml"), into)));
    REQUIRE(composed != nullptr);

    const config::configurable *routed = composed->windows[1]->as_configurable();
    REQUIRE(routed != nullptr);
    REQUIRE(routed->settings_path() == presets::screw_table_path);
    REQUIRE(config::save(into, routed->settings_edits(config::load_or_defaults(into).values)).has_value());

    const rigid_motion::capabilities motions = rigid_motion::baseline();
    const expected<manipulator::screw_modeling_window::settings, config::error> read =
            presets::read_screw_table(config::load_or_defaults(into).values, presets::screw_table_path, derived_chain(described.where), motions.screw, motions.frame);
    INFO((read ? std::string() : read.error().message));
    REQUIRE(read.has_value());

    REQUIRE(read.value().screws.size() == supplied.size());
    for(std::size_t joint = 0; joint < supplied.size(); ++joint)
    {
        INFO("joint " << joint);
        REQUIRE((read.value().screws[joint] - supplied[joint]).norm() == 0.0);
    }
}

TEST_CASE("both deployed machines open the supplied-chain scenario", "[presets][windows]")
{
    for(const auto &named : {std::pair<const char *, bool>{"ur_description/urdf/ur.urdf.xacro", true}, std::pair<const char *, bool>{"kuka_kr6_support/urdf/kr6r900sixx.xacro", false}})
    {
        INFO(named.first);

        const std::optional<presets::arm_scenario> chosen = deployed_machine(named.first, named.second);
        if(!chosen)
            SKIP("no robot description is deployed for this configuration");

        opened_arm built;
        REQUIRE(composed_windows(built.open(*chosen, presets::arm_windows_modeling(*chosen, presets::screw_table_source{}))) == modeling_windows());
        REQUIRE(drawn_axes(*built.scene) == 6u);
    }
}

// The subject of this scenario is at the flange too, and it composes no tool window either.
TEST_CASE("the supplied-chain scenario carries a frame marker at the flange and neither model", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    opened_arm built;
    manipulator::loadable_robot_stencil &drawn = drawn_by(built.open(chosen, presets::arm_windows_modeling(chosen, presets::screw_table_source{})));

    REQUIRE(drawn.attached_at(manipulator::flange_attachment::frame_marker) != nullptr);
    CHECK(drawn.attached_at(manipulator::flange_attachment::tool) == nullptr);
    CHECK(drawn.world_object() == nullptr);
}

// The marker is carried at the flange the description puts there and the chain's far end is the
// place the typed home pose names. A chain that is the derived one puts the two together, which is
// what makes the gap between them a reading rather than a claim about pixels.
TEST_CASE("a supplied chain that is the derived one ends where the frame marker stands", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen     = described_by(described.where);
    const manipulator::screw_chain derived = derived_chain(described.where);

    const presets::screw_table_source supplied =
            supplied_from(kept_chain(derived.space_screws, derived.home.block<3, 1>(0, 3), "marker-derived.xml"), chain_binding("marker-derived-into.xml"));

    opened_arm built;
    const std::shared_ptr<scene::preset> composed = built.open(chosen, presets::arm_windows_modeling(chosen, supplied));
    built.draw(*composed);

    const std::shared_ptr<threepp::Object3D> put = frame_marker_of(composed);
    REQUIRE(put != nullptr);

    const std::vector<Eigen::Vector3d> points = supplied_chain_points(*built.scene);
    REQUIRE(points.size() == 8u);
    CHECK((mark_in_world(*put) - points.back()).norm() < read_back);
}

// The marker is where the arm is and the chain's end is where the model says it is, so a home pose
// typed somewhere the flange is not separates the two by exactly what was typed wrong.
TEST_CASE("a supplied chain whose home pose is displaced ends that far from the frame marker", "[presets][windows]")
{
    const Eigen::Vector3d displaced(0.0, 0.0, 0.25);

    const described_arm described(6, "six");
    const presets::arm_scenario chosen     = described_by(described.where);
    const manipulator::screw_chain derived = derived_chain(described.where);

    const presets::screw_table_source supplied =
            supplied_from(kept_chain(derived.space_screws, derived.home.block<3, 1>(0, 3) + displaced, "marker-displaced.xml"), chain_binding("marker-displaced-into.xml"));

    opened_arm built;
    const std::shared_ptr<scene::preset> composed = built.open(chosen, presets::arm_windows_modeling(chosen, supplied));
    built.draw(*composed);

    const std::shared_ptr<threepp::Object3D> put = frame_marker_of(composed);
    REQUIRE(put != nullptr);

    const std::vector<Eigen::Vector3d> points = supplied_chain_points(*built.scene);
    REQUIRE(points.size() == 8u);
    CHECK(((points.back() - mark_in_world(*put)) - displaced).norm() < read_back);
}

// The other direction of the same rule: a composition offering the windows that hide the two models
// is the composition that draws them, and the declaration decides both directions at once.
TEST_CASE("the compositions that compose a tool window and a world-object window draw both models", "[presets][windows]")
{
    const described_arm described(6, "six");
    const written_model tool("praxis_scenarios_tool.stl", 0.1f);
    const written_model world("praxis_scenarios_world.stl", 0.2f);
    const presets::arm_scenario chosen = carrying_models(described.where, tool.where, world.where);

    opened_arm shown;
    manipulator::loadable_robot_stencil &demonstrated = drawn_by(shown.open(chosen, presets::arm_windows(chosen)));
    CHECK(demonstrated.attached_at(manipulator::flange_attachment::tool) != nullptr);
    CHECK(demonstrated.world_object() != nullptr);

    opened_arm tooled;
    manipulator::loadable_robot_stencil &carried = drawn_by(tooled.open(chosen, presets::arm_windows_tooling(chosen)));
    CHECK(carried.attached_at(manipulator::flange_attachment::tool) != nullptr);
    CHECK(carried.world_object() != nullptr);
}

// The window's offer on leaving is what arms the leave prompt, so a document just written from this
// window has to report nothing outstanding against it. An offer that can never converge raises the
// prompt from the first frame with no user action, and a person who chose to keep on leaving writes
// the file on every unload.
TEST_CASE("a chain window saved into its own document reports nothing left to decide", "[presets][configuration]")
{
    const described_arm described(6, "six_converged");
    const presets::arm_scenario chosen     = described_by(described.where);
    const std::vector<screw_axis> supplied = a_supplied_chain(6);
    const config::binding into             = chain_binding("window-converged.xml");

    opened_arm built;
    const std::shared_ptr<scene::preset> composed = built.open(chosen, presets::arm_windows_modeling(chosen, supplied_from(kept_chain(supplied, "window-converged-source.xml"), into)));
    REQUIRE(composed != nullptr);

    const config::configurable *routed = composed->windows[1]->as_configurable();
    REQUIRE(routed != nullptr);
    REQUIRE(config::save(into, routed->settings_edits(config::load_or_defaults(into).values)).has_value());

    CHECK(routed->settings_edits(config::load_or_defaults(into).values).empty());
}
