#include "fixtures.h"
#include "window_stage.h"

#include "praxis/manipulator/edited_pose.h"
#include "praxis/manipulator/control_mode.h"
#include "praxis/manipulator/screw_jog_window.h"
#include "praxis/manipulator/control_configuration.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

constexpr std::string_view screw_jog_at = "machine/screw_jog";

const rigid_motion::frame_ops reference = rigid_motion::baseline().frame;

config::declaration described()
{
    config::declaration shape("probe");
    shape.group("machine");
    declare_screw_jog(shape, screw_jog_at);
    return shape;
}

// `body` is written between the machine element's tags; with no body the file is left absent.
config::location authored(const std::string &name, const std::string &body = std::string())
{
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "praxis-screw-jog-configuration";
    std::filesystem::create_directories(directory);
    std::filesystem::remove(directory / name);
    if(!body.empty())
        std::ofstream(directory / name, std::ios::binary | std::ios::trunc) << "<probe><machine>" << body << "</machine></probe>\n";

    return config::resolve(directory / name, directory);
}

config::location starter_at(const std::string &name)
{
    const config::location at = authored(name);
    REQUIRE(config::write_template(described(), at.resolved).has_value());
    return at;
}

config::document read(const config::location &at)
{
    const config::outcome answered = config::load_or_defaults(described(), at, config::expectation::partial);
    INFO((answered.failure ? answered.failure->message : std::string()));
    REQUIRE_FALSE(answered.failure.has_value());
    return answered.values;
}

void save(const config::location &at, const std::vector<config::edit> &changes)
{
    const expected<void, config::error> saved = config::save(described(), at, changes);
    INFO((saved ? std::string() : saved.error().message));
    REQUIRE(saved.has_value());
}

// Every value apart from the default, and none of them a value another stands at.
screw_jog_window::settings moved_screw()
{
    return screw_jog_window::settings{control_mode::preview, Eigen::Vector3f{0.1f, 0.f, 0.2f}, Eigen::Vector3f{0.f, 1.f, 0.f}, 0.05f, 30.f};
}

void require_same(const screw_jog_window::settings &read, const screw_jog_window::settings &written)
{
    CHECK(read.mode == written.mode);
    CHECK(read.q == written.q);
    CHECK(read.w == written.w);
    CHECK(read.pitch == written.pitch);
    CHECK(read.theta_degrees == written.theta_degrees);
}

arm_reader published_arm()
{
    static const std::shared_ptr<arm_publisher> published = publishing(at_rest(configuration(0.0, 0.0), Eigen::Vector3d::Zero(), rotation::Identity()));
    return published->reader();
}

std::unique_ptr<screw_jog_window> window_over(const screw_jog_window::settings &state)
{
    return std::make_unique<screw_jog_window>("Screw jog", published_arm(), std::weak_ptr<owned_arm>(), reference, std::make_shared<edited_pose>(), state, std::string(screw_jog_at),
                                              std::string(), std::string(screw_jog_at));
}

std::unique_ptr<screw_jog_window> window_at_its_path_alone(const screw_jog_window::settings &state)
{
    return std::make_unique<screw_jog_window>("Screw jog", published_arm(), std::weak_ptr<owned_arm>(), reference, std::make_shared<edited_pose>(), state, std::string(screw_jog_at));
}

// The pane's last row carries the reset control; the pitch stands two rows above it.
void type_rows_above(imgui_frame &frames, const drawing &draw, int rows, const char *text)
{
    reach(frames, draw, ImGuiKey_End);
    for(int row = 0; row < rows; ++row)
        tap(frames, draw, ImGuiKey_UpArrow);
    type_at_cursor(frames, draw, text);
}

void press_reset(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_Space);
}

bool offers_only_pitch(const std::vector<config::edit> &offered, const std::string &value)
{
    return offered.size() == 1u && offered.front().key == "machine/screw_jog/pitch" && offered.front().value == value;
}

// The start tag the saved text opens with `opening`, up to its closing bracket.
std::string start_tag(const std::string &saved, const std::string &opening)
{
    const std::size_t from = saved.find(opening);
    return from == std::string::npos ? std::string() : saved.substr(from, saved.find('>', from) - from);
}

}

TEST_CASE("a screw held by the screw jog window is offered and saved and a window opened from the saved document holds it", "[manipulator][configuration]")
{
    const config::location at = starter_at("held-screw.xml");

    const std::vector<config::edit> offered = window_over(moved_screw())->settings_edits(read(at));
    REQUIRE_FALSE(offered.empty());
    save(at, offered);

    const config::document reloaded         = read(at);
    const screw_jog_window::settings reread = read_screw_jog(reloaded, screw_jog_at);
    require_same(reread, moved_screw());

    const std::unique_ptr<screw_jog_window> reopened = window_over(reread);
    CHECK(reopened->settings_edits(reloaded).empty());
    require_same(reopened->state(), reread);
}

TEST_CASE("a screw jog window standing at what its document gave offers nothing", "[manipulator][configuration]")
{
    const config::document absent = config::load_or_defaults(described(), authored("untouched-absent.xml"), config::expectation::partial).values;

    for(const config::document &carried : {read(starter_at("untouched-starter.xml")), absent})
        CHECK(window_over(read_screw_jog(carried, screw_jog_at))->settings_edits(carried).empty());
}

TEST_CASE("a screw jog window offers only the screw values somebody moved", "[manipulator][configuration]")
{
    const config::document carried                = read(authored("moved-pitch.xml", "<screw_jog mode=\"preview\"/>"));
    const std::unique_ptr<screw_jog_window> panel = window_over(read_screw_jog(carried, screw_jog_at));

    imgui_frame frames;
    const drawing draw = over(*panel);
    start_navigating(frames, draw);
    type_rows_above(frames, draw, 2, "0.25");
    CHECK(offers_only_pitch(panel->settings_edits(carried), "0.25"));

    type_rows_above(frames, draw, 2, "0");
    CHECK(panel->settings_edits(carried).empty());
}

TEST_CASE("a screw jog window reset offers the default screw back over a document carrying another", "[manipulator][configuration]")
{
    const config::document carried                = read(authored("reset-pitch.xml", "<screw_jog mode=\"preview\" pitch=\"0.5\"/>"));
    const std::unique_ptr<screw_jog_window> panel = window_over(read_screw_jog(carried, screw_jog_at));
    REQUIRE(panel->state().pitch == 0.5f);
    REQUIRE(panel->settings_edits(carried).empty());

    imgui_frame frames;
    const drawing draw = over(*panel);
    start_navigating(frames, draw);
    press_reset(frames, draw);

    CHECK(offers_only_pitch(panel->settings_edits(carried), "0"));
}

TEST_CASE("the screw jog's pitch and angle are attributes of its element and its point and direction are elements under it", "[manipulator][configuration]")
{
    const config::location at = authored("screw-layout.xml", "<screw_jog mode=\"simulation\"/>");
    save(at, write_screw_jog(moved_screw(), screw_jog_at));

    std::ifstream in(at.resolved, std::ios::binary);
    const std::string saved{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    INFO(saved);

    CHECK(start_tag(saved, "<screw_jog ").find(" pitch=\"0.05\"") != std::string::npos);
    CHECK(start_tag(saved, "<screw_jog ").find(" theta=\"30\"") != std::string::npos);
    CHECK(start_tag(saved, "<q ").find(" x=\"0.1\"") != std::string::npos);
    CHECK(start_tag(saved, "<w ").find(" y=\"1\"") != std::string::npos);
    CHECK(saved.find("q_x") == std::string::npos);
    CHECK(saved.find("pitch_") == std::string::npos);
}

TEST_CASE("a document naming no screw values opens the screw jog at the z axis through the origin", "[manipulator][configuration]")
{
    const screw_jog_window::settings read_back = read_screw_jog(read(authored("mode-only.xml", "<screw_jog mode=\"preview\"/>")), screw_jog_at);

    CHECK(read_back.mode == control_mode::preview);
    CHECK(read_back.q.isZero());
    CHECK(read_back.w == Eigen::Vector3f::UnitZ());
    CHECK(read_back.pitch == 0.f);
    CHECK(read_back.theta_degrees == 0.f);
}

TEST_CASE("a zero direction saved reads back zero", "[manipulator][configuration]")
{
    const config::location at = starter_at("zero-direction.xml");
    const screw_jog_window::settings directionless{control_mode::simulation, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()};

    save(at, window_over(directionless)->settings_edits(read(at)));
    const screw_jog_window::settings reread = read_screw_jog(read(at), screw_jog_at);

    CHECK(reread.w.isZero());
    CHECK(window_over(reread)->state().w.isZero());
}

TEST_CASE("a screw jog window given no screw path offers its mode alone and keeps its screw for the run", "[manipulator][configuration]")
{
    const config::location at                     = starter_at("mode-alone.xml");
    const std::unique_ptr<screw_jog_window> panel = window_at_its_path_alone(moved_screw());

    const std::vector<config::edit> offered = panel->settings_edits(read(at));
    REQUIRE(offered.size() == 1u);
    CHECK(offered.front().key == "machine/screw_jog/mode");
    CHECK(offered.front().value == "preview");

    save(at, offered);
    require_same(read_screw_jog(read(at), screw_jog_at), screw_jog_window::settings{control_mode::preview});
    require_same(panel->state(), moved_screw());
}

TEST_CASE("a screw jog window given no key path offers nothing", "[manipulator][configuration]")
{
    const screw_jog_window unnamed("Screw jog", published_arm(), std::weak_ptr<owned_arm>(), reference, std::make_shared<edited_pose>(), moved_screw());
    const screw_jog_window screw_alone("Screw jog", published_arm(), std::weak_ptr<owned_arm>(), reference, std::make_shared<edited_pose>(), moved_screw(), std::string(),
                                       std::string("tool_pose"), std::string(screw_jog_at));

    CHECK(unnamed.as_configurable() == nullptr);
    CHECK(screw_alone.as_configurable() == nullptr);
}

TEST_CASE("a screw jog window given no screw path leaves the screw its document carries", "[manipulator][configuration]")
{
    const config::document carried                = read(authored("carried-pitch.xml", "<screw_jog mode=\"preview\" pitch=\"0.5\"/>"));
    const std::unique_ptr<screw_jog_window> panel = window_at_its_path_alone(read_screw_jog(carried, screw_jog_at));
    REQUIRE(panel->state().pitch == 0.5f);
    REQUIRE(panel->settings_edits(carried).empty());

    imgui_frame frames;
    const drawing draw = over(*panel);
    start_navigating(frames, draw);
    press_reset(frames, draw);

    CHECK(panel->state().pitch == 0.f);
    CHECK(panel->settings_edits(carried).empty());
}

TEST_CASE("the screw jog's mapping writes the mode alone where no screw path is named and the whole element where both paths are alike", "[manipulator][configuration]")
{
    const std::vector<config::edit> mode_alone = write_screw_jog(moved_screw(), screw_jog_at, "");
    REQUIRE(mode_alone.size() == 1u);
    CHECK(mode_alone.front().key == "machine/screw_jog/mode");
    CHECK(mode_alone.front().value == "preview");

    const std::vector<config::edit> both_alike = write_screw_jog(moved_screw(), screw_jog_at, screw_jog_at);
    const std::vector<config::edit> whole      = write_screw_jog(moved_screw(), screw_jog_at);
    REQUIRE(both_alike.size() == whole.size());
    for(std::size_t at = 0; at < whole.size(); ++at)
    {
        CHECK(both_alike[at].key == whole[at].key);
        CHECK(both_alike[at].value == whole[at].value);
    }
}
