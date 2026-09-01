#include "velocity_kinematics_stage.h"

#include "../presets/scratch_directory.h"

#include "praxis/manipulator/render_configuration.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"
#include "praxis/config/configurable.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>
#include <fstream>
#include <optional>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

using opening = render_controls_window::settings;

constexpr std::string_view render_at = "machine/render_controls";

config::declaration described()
{
    config::declaration shape("probe");
    shape.group("machine");
    declare_render_controls(shape, render_at);

    return shape;
}

config::document loaded(const config::location &at, config::expectation carries = config::expectation::complete)
{
    const config::outcome answered = config::load_or_defaults(described(), at, carries);
    INFO((answered.failure ? answered.failure->message : std::string()));
    REQUIRE_FALSE(answered.failure.has_value());

    return answered.values;
}

config::document carrying(const std::string &name, std::string_view body)
{
    const std::filesystem::path where = shared_scratch_directory() / name;
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << "<probe><machine>" << body << "</machine></probe>\n";
    out.close();

    return loaded(config::resolve(where, shared_scratch_directory()), config::expectation::partial);
}

config::location cleared(const std::string &name)
{
    const std::filesystem::path where = shared_scratch_directory() / name;
    std::filesystem::remove(where);
    REQUIRE(config::write_template(described(), where).has_value());

    return config::resolve(where, shared_scratch_directory());
}

config::document saved_and_reloaded(const config::location &at, const std::vector<config::edit> &changes)
{
    const expected<void, config::error> saved = config::save(described(), at, changes);
    INFO((saved ? std::string() : saved.error().message));
    REQUIRE(saved.has_value());

    return loaded(at);
}

// Every length moved off what the settings struct opens at, in the order that struct declares them,
// so a value that failed to travel reads back as absence rather than as what was asked for.
opening every_length_moved()
{
    return opening{0.05, 0.125, 0.25, 0.375, 0.5};
}

void values_absent(const opening &read)
{
    CHECK_FALSE(read.angular_scale.has_value());
    CHECK_FALSE(read.linear_scale.has_value());
    CHECK_FALSE(read.angular_column_scale.has_value());
    CHECK_FALSE(read.linear_column_scale.has_value());
    CHECK_FALSE(read.force_cap_ratio.has_value());
}

void stands_at(const opening &read, const opening &written)
{
    REQUIRE(read.angular_scale.has_value());
    REQUIRE(read.force_cap_ratio.has_value());
    CHECK(*read.angular_scale == Catch::Approx(*written.angular_scale));
    CHECK(*read.linear_scale == Catch::Approx(*written.linear_scale));
    CHECK(*read.angular_column_scale == Catch::Approx(*written.angular_column_scale));
    CHECK(*read.linear_column_scale == Catch::Approx(*written.linear_column_scale));
    CHECK(*read.force_cap_ratio == Catch::Approx(*written.force_cap_ratio));
}

}

TEST_CASE("a document naming no length yields the window at the extents its stencil opened, every value absent", "[manipulator][configuration]")
{
    values_absent(read_render_controls(carrying("render-absent.xml", ""), render_at));
}

TEST_CASE("a document naming every length yields each of them", "[manipulator][configuration]")
{
    const config::document carried = carrying("render-authored.xml",
                                              "<render_controls angular_scale=\"0.05\" linear_scale=\"0.125\" angular_column_scale=\"0.25\" "
                                              "linear_column_scale=\"0.375\" force_cap_ratio=\"0.5\"/>");

    stands_at(read_render_controls(carried, render_at), every_length_moved());
}

// A drawing of no extent draws nothing, so a document cannot make one vanish by naming it zero: the
// length reads back absent and the stencil keeps whatever it opened at.
TEST_CASE("a scale or a cut named at zero or below reads back absent", "[manipulator][configuration]")
{
    const config::document carried = carrying("render-vanishing.xml",
                                              "<render_controls angular_scale=\"0\" linear_scale=\"-1.5\" angular_column_scale=\"0\" "
                                              "linear_column_scale=\"-0.25\" force_cap_ratio=\"0\"/>");

    values_absent(read_render_controls(carried, render_at));
}

TEST_CASE("every length written through the declared keys reads back as it was set", "[manipulator][configuration]")
{
    const opening written = every_length_moved();

    stands_at(read_render_controls(saved_and_reloaded(cleared("render-written.xml"), write_render_controls(written, render_at)), render_at), written);
}

// The window is what a learner moves a length through, so the edits it offers are what has to reach
// the document; a window offering none saves nothing however well the writer works.
TEST_CASE("a window's own lengths reach a document through the edits it offers, and one standing at what that document carries offers none", "[manipulator][configuration]")
{
    velocity_stage headless;
    const config::location at = cleared("render-window.xml");
    const render_controls_window panel("Render controls", headless.shown, render_controls_window::controls{}, every_length_moved(), std::string(render_at));

    REQUIRE(panel.as_configurable() != nullptr);
    const std::vector<config::edit> offered = panel.as_configurable()->settings_edits(loaded(at));
    REQUIRE_FALSE(offered.empty());

    const config::document written = saved_and_reloaded(at, offered);
    stands_at(read_render_controls(written, render_at), every_length_moved());

    const render_controls_window standing("Render controls", headless.shown, render_controls_window::controls{}, read_render_controls(written, render_at), std::string(render_at));

    REQUIRE(standing.as_configurable() != nullptr);
    CHECK(standing.as_configurable()->settings_edits(written).empty());
}
