#include "fixtures.h"
#include "window_stage.h"

#include "../presets/scratch_directory.h"

#include "configuration_keys.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/edited_pose.h"
#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/screw_jog_window.h"
#include "praxis/manipulator/robot_view_window.h"
#include "praxis/manipulator/pose_configuration.h"
#include "praxis/manipulator/tool_configuration.h"
#include "praxis/manipulator/view_configuration.h"
#include "praxis/manipulator/world_object_window.h"
#include "praxis/manipulator/render_configuration.h"
#include "praxis/manipulator/control_configuration.h"
#include "praxis/manipulator/loadable_robot_stencil.h"
#include "praxis/manipulator/render_controls_window.h"
#include "praxis/manipulator/control_parameters_window.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"
#include "praxis/config/configurable.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <cmath>
#include <array>
#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

constexpr std::string_view view_at       = "machine/robot_view";
constexpr std::string_view tool_at       = "machine/tool";
constexpr std::string_view screw_jog_at  = "machine/screw_jog";
constexpr std::string_view pose_at       = "machine/pose";
constexpr std::string_view world_at      = "machine/world_object";
constexpr std::string_view render_at     = "machine/render_controls";
constexpr std::string_view parameters_at = "machine/parameters";

constexpr const char *hand_written_scale  = "<robot_view model=\"Meshes\" frame_marker_scale=\"0.123456789\"/>";
constexpr const char *hand_written_offset = "<tool><kinematics><offset x=\"0.123456789\"/></kinematics></tool>";

config::declaration described()
{
    config::declaration shape("probe");
    shape.group("machine");
    declare_robot_view(shape, view_at);
    declare_tool(shape, tool_at);
    declare_screw_jog(shape, screw_jog_at);
    declare_shared_pose(shape, pose_at);
    declare_world_object(shape, world_at);
    declare_render_controls(shape, render_at);
    declare_control_parameters(shape, parameters_at);

    return shape;
}

config::location authored(const std::string &name, std::string_view body)
{
    const std::filesystem::path where = shared_scratch_directory() / name;
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << "<probe><machine>" << body << "</machine></probe>\n";
    out.close();

    return config::resolve(where, shared_scratch_directory());
}

config::document loaded(const config::location &at)
{
    const config::outcome answered = config::load_or_defaults(described(), at, config::expectation::partial);
    INFO((answered.failure ? answered.failure->message : std::string()));
    REQUIRE_FALSE(answered.failure.has_value());

    return answered.values;
}

std::string file_text(const config::location &at)
{
    std::ifstream in(at.resolved, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

void saved(const config::location &at, const std::vector<config::edit> &changes)
{
    const expected<void, config::error> written = config::save(described(), at, changes);
    INFO((written ? std::string() : written.error().message));
    REQUIRE(written.has_value());
}

bool offers_key(const std::vector<config::edit> &offered, const std::string &key)
{
    return std::ranges::find(offered, key, &config::edit::key) != offered.end();
}

std::string offered_at(const std::vector<config::edit> &offered, std::string_view at, std::string_view leaf)
{
    const auto found = std::ranges::find(offered, std::string(at) + "/" + std::string(leaf), &config::edit::key);
    return found != offered.end() ? found->value : std::string();
}

struct stage
{
    stage()
            : loop(scheduler::inline_workers)
            , scene(threepp::Scene::create())
            , published(std::make_shared<arm_publisher>())
            , shown(two_joint_handle(), attachments{}, *scene, loop.main_strand(), published->reader(), rigid_motion::baseline().screw, rigid_motion::screw_slot_set{})
    {
    }

    scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<arm_publisher> published;
    loadable_robot_stencil shown;
};

robot_view_window view_over(stage &headless, const robot_view_window::settings &state)
{
    return robot_view_window("Arm view", headless.shown, robot_view_window::controls(), state, std::string(view_at));
}

std::unique_ptr<tool_window> tool_over(stage &headless, const tool_window::settings &state)
{
    auto panel = std::make_unique<tool_window>("Tool settings", headless.shown, headless.published->reader(), std::weak_ptr<owned_arm>(), rigid_motion::baseline().frame, state,
                                               std::string(tool_at));
    panel->initialize();
    return panel;
}

std::unique_ptr<screw_jog_window> screw_jog_over(const screw_jog_window::settings &state)
{
    static const std::shared_ptr<arm_publisher> published = publishing(at_rest(configuration(0.0, 0.0), Eigen::Vector3d::Zero(), rotation::Identity()));

    return std::make_unique<screw_jog_window>("Screw jog", published->reader(), std::weak_ptr<owned_arm>(), rigid_motion::baseline().frame, std::make_shared<edited_pose>(), state,
                                              std::string(screw_jog_at), std::string(), screw_jog_window::screw_keeping::with_document);
}

world_object_window world_over(stage &headless, const world_object_window::settings &state)
{
    return world_object_window("World object", headless.shown, rigid_motion::baseline().frame, state, std::string(world_at));
}

render_controls_window render_over(stage &headless, const render_controls_window::settings &state)
{
    return render_controls_window("Render controls", headless.shown, render_controls_window::controls{}, state, std::string(render_at));
}

control_parameters_window parameters_over(stage &headless, const control_parameters_window::settings &state)
{
    return control_parameters_window("Control parameters", headless.published->reader(), std::weak_ptr<owned_arm>(), state, std::string(parameters_at));
}

bool unsaved(const config::configurable *panel, const config::document &carried)
{
    const std::array<const config::configurable *, 1> shown{panel};
    return config::anything_unsaved(shown, carried);
}

}

TEST_CASE("a view window over a hand-written frame marker scale its float cannot hold opens with nothing unsaved and keeps the text through an unrelated save",
          "[manipulator][configuration]")
{
    const config::location at      = authored("held-scale-kept.xml", hand_written_scale);
    const config::document carried = loaded(at);
    stage headless;

    const robot_view_window opened = view_over(headless, read_robot_view(carried, view_at));
    REQUIRE(opened.state().marker_scale != 0.123456789);
    CHECK(opened.settings_edits(carried).empty());
    CHECK_FALSE(unsaved(opened.as_configurable(), carried));

    robot_view_window::settings changed     = read_robot_view(carried, view_at);
    changed.model                           = model_render::chain;
    const std::vector<config::edit> offered = view_over(headless, changed).settings_edits(carried);
    CHECK(offers_key(offered, std::string(view_at) + "/model"));
    CHECK_FALSE(offers_key(offered, std::string(view_at) + "/frame_marker_scale"));

    saved(at, offered);
    CHECK(file_text(at).find("frame_marker_scale=\"0.123456789\"") != std::string::npos);
    CHECK(file_text(at).find("model=\"Joint chain\"") != std::string::npos);

    const config::document reread = loaded(at);
    CHECK(view_over(headless, read_robot_view(reread, view_at)).settings_edits(reread).empty());
}

TEST_CASE("a frame marker scale edited in the view window is written at the precision the window holds", "[manipulator][configuration]")
{
    const config::location at      = authored("held-scale-edited.xml", hand_written_scale);
    const config::document carried = loaded(at);
    const std::string scale_key    = std::string(view_at) + "/frame_marker_scale";
    stage headless;

    robot_view_window::settings edited      = read_robot_view(carried, view_at);
    edited.marker_scale                     = 1.0 / 3.0;
    const std::vector<config::edit> offered = view_over(headless, edited).settings_edits(carried);
    const auto scale                        = std::ranges::find(offered, scale_key, &config::edit::key);
    REQUIRE(scale != offered.end());
    CHECK(scale->value == "0.33333334");

    saved(at, offered);
    CHECK(file_text(at).find("frame_marker_scale=\"0.33333334\"") != std::string::npos);
}

TEST_CASE("a tool window over a hand-written offset component its float cannot hold opens with nothing unsaved and keeps the text through an unrelated save",
          "[manipulator][configuration]")
{
    const config::location at      = authored("held-offset-kept.xml", hand_written_offset);
    const config::document carried = loaded(at);
    stage headless;

    const tool_window::settings opening = read_tool(carried, tool_at);
    REQUIRE(opening.kinematics_offset.x() != 0.123456789);
    CHECK(tool_over(headless, opening)->settings_edits(carried).empty());
    CHECK_FALSE(unsaved(tool_over(headless, opening)->as_configurable(), carried));

    tool_window::settings changed           = opening;
    changed.kinematics_offset.z()           = 0.5f;
    const std::vector<config::edit> offered = tool_over(headless, changed)->settings_edits(carried);
    CHECK(offers_key(offered, std::string(tool_at) + "/kinematics/offset/z"));
    CHECK_FALSE(offers_key(offered, std::string(tool_at) + "/kinematics/offset/x"));

    saved(at, offered);
    CHECK(file_text(at).find("x=\"0.123456789\"") != std::string::npos);
    CHECK(file_text(at).find("z=\"0.5\"") != std::string::npos);

    const config::document reread = loaded(at);
    CHECK(tool_over(headless, read_tool(reread, tool_at))->settings_edits(reread).empty());
}

TEST_CASE("an offset component edited in the tool window is written at the precision the window holds", "[manipulator][configuration]")
{
    const config::location at      = authored("held-offset-edited.xml", hand_written_offset);
    const config::document carried = loaded(at);
    const std::string x_key        = std::string(tool_at) + "/kinematics/offset/x";
    stage headless;

    tool_window::settings edited            = read_tool(carried, tool_at);
    edited.kinematics_offset.x()            = 1.0f / 3.0f;
    const std::vector<config::edit> offered = tool_over(headless, edited)->settings_edits(carried);
    const auto x                            = std::ranges::find(offered, x_key, &config::edit::key);
    REQUIRE(x != offered.end());
    CHECK(x->value == "0.33333334");

    saved(at, offered);
    CHECK(file_text(at).find("x=\"0.33333334\"") != std::string::npos);
}

TEST_CASE("a held real is offered as the document's own text exactly where the document's value rounds to it", "[manipulator][configuration]")
{
    const double written = 0.123456789;
    const float held     = static_cast<float>(written);
    const float above    = std::nextafter(held, std::numeric_limits<float>::infinity());
    const float below    = std::nextafter(held, -std::numeric_limits<float>::infinity());

    CHECK(keys::held_text(keys::text_of(held), written, "0.123456789") == "0.123456789");
    CHECK(keys::held_text(keys::text_of(above), written, "0.123456789") == keys::text_of(above));
    CHECK(keys::held_text(keys::text_of(below), written, "0.123456789") == keys::text_of(below));

    const config::document carried = loaded(authored("held-scale-read.xml", hand_written_scale));
    CHECK(keys::real_at(carried, std::string(view_at) + "/frame_marker_scale", 1.0f) == held);

    const std::vector<config::edit> offered{config::edit{std::string(view_at) + "/frame_marker_scale", keys::text_of(held)}};
    CHECK(keys::held_as_carried(carried, offered).front().value == "0.123456789");
}

TEST_CASE("an offer that is not a real the document declares passes through the helper unchanged", "[manipulator][configuration]")
{
    const config::document carried = loaded(authored("held-passing.xml", hand_written_scale));
    const std::vector<config::edit> offered{
            config::edit{std::string(view_at) + "/screw_axes", "false"},
            config::edit{std::string(view_at) + "/model", "Joint chain"},
            config::edit{std::string(view_at) + "/undeclared", "0.12345679"},
            config::edit{std::string(view_at) + "/frame_marker_scale", "0.12345679", config::edit_kind::refused},
    };

    const std::vector<config::edit> passed = keys::held_as_carried(carried, offered);
    REQUIRE(passed.size() == offered.size());
    for(std::size_t at = 0u; at < offered.size(); ++at)
    {
        CHECK(passed[at].key == offered[at].key);
        CHECK(passed[at].value == offered[at].value);
        CHECK(passed[at].kind == offered[at].kind);
    }
}

TEST_CASE("a screw jog window over a hand-written point component its float cannot hold opens with nothing unsaved", "[manipulator][configuration]")
{
    const config::document carried           = loaded(authored("held-screw-point.xml", "<screw_jog><q x=\"0.123456789\" y=\"0\" z=\"0\"/></screw_jog>"));
    const screw_jog_window::settings opening = read_screw_jog(carried, screw_jog_at);
    REQUIRE(opening.q.x() != 0.123456789);
    CHECK(screw_jog_over(opening)->settings_edits(carried).empty());
}

TEST_CASE("a shared pose over a hand-written position its float cannot hold offers nothing and keeps the text through an unrelated save, and an edited one is written at "
          "float precision",
          "[manipulator][configuration]")
{
    const config::location at      = authored("held-pose-position.xml", "<pose><position x=\"0.123456789\" y=\"0\" z=\"0\"/></pose>");
    const config::document carried = loaded(at);
    edited_pose pose               = read_shared_pose(carried, pose_at);
    REQUIRE(pose.standing == pose_standing::held);
    CHECK(unsaved_shared_pose(carried, pose, pose_at).empty());

    pose.position.y() = 0.5f;
    saved(at, unsaved_shared_pose(carried, pose, pose_at));
    CHECK(file_text(at).find("x=\"0.123456789\"") != std::string::npos);
    CHECK(file_text(at).find("y=\"0.5\"") != std::string::npos);

    pose.position.x() = 1.0f / 3.0f;
    CHECK(offered_at(unsaved_shared_pose(carried, pose, pose_at), pose_at, "position/x") == "0.33333334");
}

TEST_CASE("a world object window over a hand-written offset component its float cannot hold opens with nothing unsaved", "[manipulator][configuration]")
{
    const config::document carried = loaded(authored("held-world-offset.xml", "<world_object><offset x=\"0.123456789\" y=\"0\" z=\"0\"/></world_object>"));
    stage headless;

    world_object_window::settings state = read_world_object(carried, world_at);
    CHECK(world_over(headless, state).settings_edits(carried).empty());

    state.gfx_offset.x() = 1.0f / 3.0f;
    CHECK(offered_at(world_over(headless, state).settings_edits(carried), world_at, "offset/x") == "0.33333334");
}

TEST_CASE("a render controls window over a hand-written length its float cannot hold opens with nothing unsaved", "[manipulator][configuration]")
{
    const config::document carried = loaded(authored("held-render-length.xml", "<render_controls linear_scale=\"0.123456789\"/>"));
    stage headless;

    render_controls_window::settings state = read_render_controls(carried, render_at);
    REQUIRE(state.linear_scale.has_value());
    CHECK(render_over(headless, state).settings_edits(carried).empty());

    state.linear_scale = 1.0 / 3.0;
    CHECK(offered_at(render_over(headless, state).settings_edits(carried), render_at, "linear_scale") == "0.33333334");
}

TEST_CASE("a control parameters window over a hand-written velocity factor its float cannot hold opens with nothing unsaved", "[manipulator][configuration]")
{
    const config::document carried = loaded(authored("held-velocity-factor.xml", "<parameters velocity_factor=\"0.123456789\"/>"));
    stage headless;

    control_parameters_window::settings state = read_control_parameters(carried, parameters_at);
    CHECK(parameters_over(headless, state).settings_edits(carried).empty());

    state.velocity = 1.0f / 3.0f;
    CHECK(offered_at(parameters_over(headless, state).settings_edits(carried), parameters_at, "velocity_factor") == "0.33333334");
}
