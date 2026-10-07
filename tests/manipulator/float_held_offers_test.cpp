#include "fixtures.h"

#include "../presets/scratch_directory.h"

#include "configuration_keys.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/robot_view_window.h"
#include "praxis/manipulator/tool_configuration.h"
#include "praxis/manipulator/view_configuration.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

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

constexpr std::string_view view_at = "machine/robot_view";
constexpr std::string_view tool_at = "machine/tool";

constexpr const char *hand_written_scale  = "<robot_view model=\"Meshes\" frame_marker_scale=\"0.123456789\"/>";
constexpr const char *hand_written_offset = "<tool><kinematics><offset x=\"0.123456789\"/></kinematics></tool>";

config::declaration described()
{
    config::declaration shape("probe");
    shape.group("machine");
    declare_robot_view(shape, view_at);
    declare_tool(shape, tool_at);

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
