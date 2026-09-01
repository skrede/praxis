#include "window_stage.h"
#include "drawn_chain.h"

#include "../presets/drawn_lines.h"

#include "praxis/manipulator/configuration.h"
#include "praxis/manipulator/robot_view_window.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <imgui.h>

#include <Eigen/Core>

#include <set>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <optional>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

using controls = robot_view_window::controls;
using opening  = robot_view_window::settings;

constexpr const char *panel_title  = "Arm view";
constexpr std::string_view view_at = "machine/view";

// The half-length an arm carrying no drawn geometry opens at, in metres, and what a point read back
// out of a float buffer is comparable at.
constexpr double opening_reach = 1.0;
constexpr double read_back     = 1.0e-4;

// Inside no widget's own increment and equal to neither the opening reach nor zero, so a reach found
// at this value was typed into the control.
constexpr const char *typed_reach = "0.25";
constexpr double reached          = 0.25;

config::declaration described()
{
    config::declaration shape("probe");
    shape.group("machine");
    declare_robot_view(shape, view_at);

    return shape;
}

std::filesystem::path scratch()
{
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "praxis-robot-view-window";
    std::filesystem::create_directories(directory);

    return directory;
}

config::document starter(const std::string &name)
{
    const std::filesystem::path where = scratch() / name;
    std::filesystem::remove(where);
    REQUIRE(config::write_template(described(), where).has_value());

    const config::outcome answered = config::load_or_defaults(described(), config::resolve(where, scratch()));
    REQUIRE_FALSE(answered.failure.has_value());

    return answered.values;
}

config::document saved_and_reloaded(const std::string &name, const std::vector<config::edit> &changes)
{
    const std::filesystem::path where = scratch() / name;
    std::filesystem::remove(where);
    REQUIRE(config::write_template(described(), where).has_value());

    const config::location at_file = config::resolve(where, scratch());
    REQUIRE(config::save(described(), at_file, changes).has_value());

    const config::outcome answered = config::load_or_defaults(described(), at_file);
    REQUIRE_FALSE(answered.failure.has_value());

    return answered.values;
}

config::outcome answering(const std::string &name, std::string_view body)
{
    const std::filesystem::path where = scratch() / name;
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << "<probe><machine>" << body << "</machine></probe>\n";
    out.close();

    return config::load_or_defaults(described(), config::resolve(where, scratch()), config::expectation::partial);
}

config::document carrying(const std::string &name, std::string_view body)
{
    const config::outcome answered = answering(name, body);
    REQUIRE_FALSE(answered.failure.has_value());

    return answered.values;
}

void reads_as(const opening &read, const opening &written)
{
    CHECK(read.model == written.model);
    CHECK(read.decoration == written.decoration);
    CHECK(read.axis_reach == written.axis_reach);
}

arm_snapshot upright()
{
    const praxis::transform put = praxis::transform::Identity();
    const Eigen::Vector3d origin(Eigen::Vector3d::Zero());
    const praxis::rotation level(praxis::rotation::Identity());

    return arm_snapshot{configuration(0.0, 0.0),
                        joint_limits{},
                        put,
                        put,
                        put,
                        origin,
                        origin,
                        level,
                        level,
                        recording_parameters{},
                        1.0,
                        false,
                        task_counters{},
                        {},
                        praxis::unexpected(refusal::not_implemented),
                        praxis::unexpected(refusal::not_implemented),
                        jacobian_manipulability{praxis::unexpected(refusal::not_implemented), praxis::unexpected(refusal::not_implemented)},
                        jacobian_manipulability{praxis::unexpected(refusal::not_implemented), praxis::unexpected(refusal::not_implemented)},
                        {},
                        {},
                        nullptr,
                        nullptr,
                        {},
                        {}};
}

praxis::screw_axis revolute_screw(const Eigen::Vector3d &through)
{
    return praxis::rigid_motion::baseline().screw.screw_axis_from_point_direction_pitch(through, Eigen::Vector3d::UnitZ(), 0.0).value();
}

std::vector<praxis::screw_axis> two_axes()
{
    return {revolute_screw(Eigen::Vector3d::Zero()), revolute_screw(Eigen::Vector3d(static_cast<double>(link_length), 0.0, 0.0))};
}

// A scene needs no graphics context and a renderer robot needs no display, so the whole stage is
// built headlessly.
struct stage
{
    // A stencil told no screws folds no chain, which is the arm a scenario that never opens the
    // screw modeling window stands over.
    explicit stage(bool told_screws = true)
            : loop(inline_workers)
            , scene(threepp::Scene::create())
            , published(std::make_shared<arm_publisher>())
            , shown(two_joint_handle(), attached_models{}, *scene, loop.main_strand(), published->reader(), praxis::rigid_motion::baseline().screw,
                    praxis::rigid_motion::screw_slot_set{})
    {
        published->publish(std::make_shared<const arm_snapshot>(upright()));
        REQUIRE(shown.initialize().has_value());
        if(told_screws)
            REQUIRE(shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
    }

    void draw()
    {
        REQUIRE(loop.main_strand().post([this] { shown.render(); }).has_value());
        REQUIRE(loop.drain().has_value());
        scene->updateMatrixWorld(true);
    }

    double axis_span()
    {
        const std::vector<Eigen::Vector3d> ends = line_in_world(*scene, loadable_robot_stencil::joint_axis_name(0));
        REQUIRE(ends.size() == 2u);

        return (ends.back() - ends.front()).norm();
    }

    threepp::Object3D *axis_node()
    {
        return first_line_under(*scene, loadable_robot_stencil::joint_axis_name(0));
    }

    threepp::Object3D *chain_node()
    {
        return praxis::fixture::chain_node(*scene, loadable_robot_stencil::chain_name());
    }

    praxis::scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<arm_publisher> published;
    loadable_robot_stencil shown;
};

controls offering(int subset)
{
    controls offered;
    offered.model      = (subset & 1) != 0;
    offered.decoration = (subset & 2) != 0;
    offered.reach      = (subset & 4) != 0;

    return offered;
}

std::size_t bare_panel()
{
    return geometry_of(
            []
            {
                ImGui::Begin(panel_title);
                ImGui::End();
            });
}

// The row a control stands on, counted from the panel's first, reached from the top so that a case
// depends on the order the panel draws in rather than on where a previous case left the cursor.
void step_to(imgui_frame &frames, const drawing &draw, int row)
{
    praxis::fixture::reach(frames, draw, ImGuiKey_Home);
    for(int step = 0; step < row; ++step)
        tap(frames, draw, ImGuiKey_DownArrow);
}

// Opens the list the control on that row draws and takes the entry at that place in it, counted
// from its first: the list opens its keyboard cursor on its first entry rather than on the entry it
// is showing, so the count is a position in the list and not a distance from the current one. The
// list is a window of its own, so a frame asking for the panel to be focused would close it again:
// the panel is focused once, on the frame the walk starts from, and every frame after that is drawn
// without the request. The keyboard state a frame is left in outlives the frame, so each entry is
// taken in a context of its own.
void choose_entry(scene::imgui_window &panel, int row, int entry)
{
    imgui_frame frames;
    const drawing draw = [&panel] { panel.render(); };

    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    frames.draw(
            [&panel]
            {
                ImGui::SetNextWindowFocus();
                panel.render();
            });

    step_to(frames, draw, row);
    tap(frames, draw, ImGuiKey_Space);
    for(int step = 0; step < entry; ++step)
        tap(frames, draw, ImGuiKey_DownArrow);
    tap(frames, draw, ImGuiKey_Space);
}

}

TEST_CASE("every view field written through the declared keys reads back as it was set", "[manipulator][configuration]")
{
    for(const model_render chosen : {model_render::meshes, model_render::chain, model_render::meshes_and_chain, model_render::none})
    {
        const opening named{chosen, false, 0.75};
        reads_as(read_robot_view(saved_and_reloaded("view-round-trip-" + std::to_string(static_cast<int>(chosen)) + ".xml", write_robot_view(named, view_at)), view_at), named);
    }

    const opening unnamed{model_render::chain, true, std::nullopt};
    reads_as(read_robot_view(saved_and_reloaded("view-unnamed-reach.xml", write_robot_view(unnamed, view_at)), view_at), unnamed);
    reads_as(read_robot_view(starter("view-starter.xml"), view_at), opening{});
}

TEST_CASE("a document naming some of the three leaves leaves the rest where the settings open", "[manipulator][configuration]")
{
    const opening read = read_robot_view(carrying("view-partial.xml", "<view model=\"Joint chain\" axis_reach=\"0.25\"/>"), view_at);

    CHECK(read.model == model_render::chain);
    CHECK(read.axis_reach == std::optional<double>(reached));
    CHECK(read.decoration == opening{}.decoration);

    reads_as(read_robot_view(carrying("view-none-named.xml", ""), view_at), opening{});
}

// The model leaf is a closed set of spellings rather than free text, and the entry set is the label
// list the dropdown draws, so the document and the control cannot disagree about what exists. A
// spelling outside the set is refused where the document is read rather than silently narrowed, and
// the values a refused document still answers are the ones the settings open at.
TEST_CASE("a document carrying a model spelling the entry set does not name is refused and reads as the opening entry", "[manipulator][configuration]")
{
    const config::outcome answered = answering("view-unnamed-entry.xml", "<view model=\"a spelling no entry carries\"/>");

    REQUIRE(answered.failure.has_value());
    CHECK(answered.failure->code == config::error_code::rejected_content);
    reads_as(read_robot_view(answered.values, view_at), opening{});
}

TEST_CASE("a window draws the controls its composition asked for and no others", "[manipulator][controls]")
{
    stage headless;

    std::set<std::size_t> panels;
    for(int subset = 0; subset < 8; ++subset)
    {
        robot_view_window panel(panel_title, headless.shown, offering(subset), opening{});
        panels.insert(geometry_of(over(panel)));
    }

    CHECK(panels.size() == 8u);

    robot_view_window offering_nothing(panel_title, headless.shown, offering(0), opening{});

    CHECK(geometry_of(over(offering_nothing)) == bare_panel());
}

// The state a composition named reaches the arm whether or not a control was drawn for it, which is
// what leaves a feature nobody offered a control for standing where the composition put it.
TEST_CASE("a composition offering no decoration control opens the decoration where it said and nothing in the window moves it", "[manipulator][controls]")
{
    stage headless;
    controls offered;
    offered.decoration = false;

    robot_view_window panel(panel_title, headless.shown, offered, opening{model_render::meshes, false});
    panel.initialize();
    headless.draw();

    REQUIRE_FALSE(drawn(headless.axis_node()));

    choose_entry(panel, 0, 1);
    headless.draw();

    CHECK_FALSE(drawn(headless.axis_node()));
    CHECK(drawn(headless.chain_node()));
}

TEST_CASE("a window no key path was named for offers nothing, and one named a path answers for itself", "[manipulator][configuration]")
{
    stage headless;
    robot_view_window unrouted(panel_title, headless.shown);

    CHECK(unrouted.settings_path().empty());
    CHECK(unrouted.as_configurable() == nullptr);

    const opening held{model_render::chain, true, 0.5};
    robot_view_window routed(panel_title, headless.shown, controls(), held, std::string(view_at));
    const config::configurable *answered = routed.as_configurable();

    REQUIRE(answered != nullptr);
    CHECK(answered->settings_path() == view_at);

    const std::vector<config::edit> offered = answered->settings_edits(starter("view-routed.xml"));

    REQUIRE_FALSE(offered.empty());
    reads_as(read_robot_view(saved_and_reloaded("view-routed.xml", offered), view_at), held);
}

TEST_CASE("every control a composition offered writes through to the scene", "[manipulator][controls]")
{
    stage headless;
    robot_view_window panel(panel_title, headless.shown, offering(7), opening{model_render::meshes, true});
    panel.initialize();
    headless.draw();

    REQUIRE(drawn(rendered_arm(*headless.scene)));
    REQUIRE(drawn(headless.axis_node()));
    REQUIRE_FALSE(drawn(headless.chain_node()));
    REQUIRE(headless.axis_span() == Catch::Approx(2.0 * opening_reach).margin(read_back));

    choose_entry(panel, 0, 1);

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    step_to(frames, draw, 1);
    tap(frames, draw, ImGuiKey_Space);
    step_to(frames, draw, 2);
    type_at_cursor(frames, draw, typed_reach);
    headless.draw();

    CHECK_FALSE(drawn(rendered_arm(*headless.scene)));
    CHECK(drawn(headless.chain_node()));
    CHECK_FALSE(drawn(headless.axis_node()));
    CHECK(headless.axis_span() == Catch::Approx(2.0 * reached).margin(read_back));
}

// One control over four entries can leave no combination standing that it does not name, because
// both writes are issued on every change: an entry that does not draw something has to have turned
// it off as well as not turned it on. The axes are moved once, by the control that owns them, and
// nothing the model control does afterwards may reach them.
TEST_CASE("each model entry leaves exactly the drawing it names, and the axes where their own control left them", "[manipulator][controls]")
{
    struct chosen_entry
    {
        int entry;
        bool meshes;
        bool chain;
    };

    stage headless;
    robot_view_window panel(panel_title, headless.shown, offering(3), opening{model_render::meshes, true});
    panel.initialize();
    headless.draw();

    REQUIRE(drawn(headless.axis_node()));

    {
        imgui_frame frames;
        const drawing draw = over(panel);
        start_navigating(frames, draw);
        step_to(frames, draw, 1);
        tap(frames, draw, ImGuiKey_Space);
    }
    headless.draw();

    REQUIRE_FALSE(drawn(headless.axis_node()));

    for(const chosen_entry &taken : {chosen_entry{0, true, false}, chosen_entry{1, false, true}, chosen_entry{2, true, true}, chosen_entry{3, false, false}})
    {
        INFO("meshes " << taken.meshes << ", chain " << taken.chain);
        choose_entry(panel, 0, taken.entry);
        headless.draw();

        CHECK(drawn(rendered_arm(*headless.scene)) == taken.meshes);
        CHECK(drawn(headless.chain_node()) == taken.chain);
        CHECK_FALSE(drawn(headless.axis_node()));
    }
}

// Two of the four entries name the chain, and an arm the stencil folded none for has none to name.
// A list is walked from its first entry and loops at its end, so the entry one step reaches and the
// entry two steps reach say together how many stand in it: over an arm carrying a chain one step
// reaches the chain, and over one carrying none it reaches the entry that draws neither drawing and
// a second step is already back at the first.
TEST_CASE("a window over an arm carrying no chain offers no entry that would draw one", "[manipulator][controls]")
{
    stage headless(false);
    robot_view_window panel(panel_title, headless.shown, offering(1), opening{model_render::meshes, true});
    panel.initialize();

    REQUIRE_FALSE(headless.shown.holds_chain());

    choose_entry(panel, 0, 1);
    CHECK(panel.state().model == model_render::none);

    choose_entry(panel, 0, 2);
    CHECK(panel.state().model == model_render::meshes);
}

// The reach a composition named nothing for is the stencil's own, proportioned to whichever arm is
// rendered. Writing that number back as an explicit value would fix one machine's proportion into
// the document and hand it to every other, so the window has to spell an unnamed reach as absence
// for as long as nobody has moved it.
TEST_CASE("a reach nobody named is offered as absence, and the document it is written into reports nothing outstanding", "[manipulator][configuration]")
{
    stage headless;
    const opening unnamed{model_render::chain, true, std::nullopt};
    robot_view_window panel(panel_title, headless.shown, controls(), unnamed, std::string(view_at));

    REQUIRE_FALSE(panel.state().axis_reach.has_value());
    reads_as(panel.state(), unnamed);

    const config::configurable *answered = panel.as_configurable();
    REQUIRE(answered != nullptr);

    const config::document written = saved_and_reloaded("view-unnamed-round-trip.xml", answered->settings_edits(starter("view-unnamed-round-trip.xml")));

    reads_as(read_robot_view(written, view_at), unnamed);
    CHECK(answered->settings_edits(written).empty());
}

// A reach somebody typed is a reach the document is meant to keep, so the sentinel stands only
// until the control moves.
TEST_CASE("a reach typed into the control is written out as the value it was typed at", "[manipulator][configuration]")
{
    stage headless;
    robot_view_window panel(panel_title, headless.shown, offering(7), opening{model_render::meshes, true}, std::string(view_at));
    panel.initialize();

    REQUIRE_FALSE(panel.state().axis_reach.has_value());

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    step_to(frames, draw, 2);
    type_at_cursor(frames, draw, typed_reach);

    REQUIRE(panel.state().axis_reach == std::optional<double>(reached));

    const config::configurable *answered = panel.as_configurable();
    REQUIRE(answered != nullptr);

    const config::document written = saved_and_reloaded("view-typed-reach.xml", answered->settings_edits(starter("view-typed-reach.xml")));

    CHECK(read_robot_view(written, view_at).axis_reach == std::optional<double>(reached));
    CHECK(answered->settings_edits(written).empty());
}

// The control admits no reach below the smallest one that draws, and a composition naming a reach
// reaches the stencil without passing that control at all.
TEST_CASE("a composition naming a reach of no length opens at the smallest one that draws", "[manipulator][controls]")
{
    stage headless;
    robot_view_window panel(panel_title, headless.shown, controls(), opening{model_render::meshes, true, 0.0});
    panel.initialize();
    headless.draw();

    CHECK(headless.axis_span() > 0.0);
    CHECK(panel.state().axis_reach.has_value());
    CHECK(*panel.state().axis_reach > 0.0);
}
