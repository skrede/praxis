#include "captured_log.h"
#include "velocity_kinematics_stage.h"

#include "../presets/scratch_directory.h"

#include "praxis/manipulator/velocity_configuration.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"
#include "praxis/config/configurable.h"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <optional>
#include <algorithm>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

using opening = velocity_kinematics_window::settings;

constexpr std::string_view velocity_at = "machine/velocity_kinematics";

config::declaration described()
{
    config::declaration shape("probe");
    shape.group("machine");
    declare_velocity_kinematics(shape, velocity_at);

    return shape;
}

config::document loaded(const config::location &at, config::expectation carries = config::expectation::complete)
{
    const config::outcome answered = config::load_or_defaults(described(), at, carries);
    INFO((answered.failure ? answered.failure->message : std::string()));
    REQUIRE_FALSE(answered.failure.has_value());

    return answered.values;
}

config::location written(const std::string &name, std::string_view body)
{
    const std::filesystem::path where = shared_scratch_directory() / name;
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << "<probe><machine>" << body << "</machine></probe>\n";
    out.close();

    return config::resolve(where, shared_scratch_directory());
}

config::outcome answering(const std::string &name, std::string_view body)
{
    return config::load_or_defaults(described(), written(name, body), config::expectation::partial);
}

config::document carrying(const std::string &name, std::string_view body)
{
    const config::outcome answered = answering(name, body);
    INFO((answered.failure ? answered.failure->message : std::string()));
    REQUIRE_FALSE(answered.failure.has_value());

    return answered.values;
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

std::string bytes_of(const config::location &at)
{
    std::ifstream in(at.resolved, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

velocity_kinematics_window window_at(velocity_stage &headless, const opening &state)
{
    return velocity_kinematics_window("Velocity kinematics", headless.source->reader(), headless.arm(), headless.shown, velocity_kinematics_window::controls{}, state,
                                      std::string(velocity_at));
}

std::vector<std::string> keys_of(const std::vector<config::edit> &edits)
{
    std::vector<std::string> keys;
    for(const config::edit &one : edits)
        keys.push_back(one.key);

    return keys;
}

bool names_a_rate_switch(const config::edit &one)
{
    return one.key.ends_with("rate_weighted") || one.key.ends_with("twist");
}

// Every field moved off what the settings struct opens at, in the order that struct declares them,
// so a value that failed to travel reads back as the opening rather than as what was asked for.
opening every_field_moved()
{
    return opening{jacobian_frame::body, ellipsoid_view::force, false, false, false, false};
}

void stands_at(const opening &read, const opening &written)
{
    CHECK(read.frame == written.frame);
    CHECK(read.reading == written.reading);
    CHECK(read.angular == written.angular);
    CHECK(read.linear == written.linear);
    CHECK(read.columns == written.columns);
    CHECK(read.capped == written.capped);
}

}

TEST_CASE("a document naming nothing yields the window at the values its settings open at", "[manipulator][configuration]")
{
    stands_at(read_velocity_kinematics(carrying("absent.xml", ""), velocity_at), opening{});
}

TEST_CASE("a document naming every field yields each of them", "[manipulator][configuration]")
{
    const config::document carried =
            carrying("authored.xml", "<velocity_kinematics frame=\"body\" reading=\"force\" angular=\"false\" linear=\"false\" columns=\"false\" capped=\"false\"/>");

    stands_at(read_velocity_kinematics(carried, velocity_at), every_field_moved());
}

// The spellings index the enumerations, so one the table does not carry would otherwise be cast to a
// value neither has. The document is refused by name, and what it yields still reads back the
// openings rather than a value neither enumeration has.
TEST_CASE("a frame or a reading the table does not spell is refused by name and reads back the opening", "[manipulator][configuration]")
{
    const config::outcome answered = answering("unspelled.xml", "<velocity_kinematics frame=\"diagonal\" reading=\"momentum\"/>");

    REQUIRE(answered.failure.has_value());
    stands_at(read_velocity_kinematics(answered.values, velocity_at), opening{});
}

TEST_CASE("a document carrying a key named for an ellipsoid alone loads every other value and names that key in a warning", "[manipulator][configuration]")
{
    std::optional<config::outcome> answered;
    const std::string warned =
            praxis::tests::reported_by([&] { answered = answering("older.xml", "<velocity_kinematics frame=\"body\" angular_ellipsoid=\"false\" linear_ellipsoid=\"false\"/>"); });

    REQUIRE(answered.has_value());
    REQUIRE_FALSE(answered->failure.has_value());
    CHECK(warned.find("angular_ellipsoid") != std::string::npos);
    CHECK(warned.find("linear_ellipsoid") != std::string::npos);

    const opening read = read_velocity_kinematics(answered->values, velocity_at);
    CHECK(read.frame == jacobian_frame::body);
    CHECK(read.angular == opening{}.angular);
    CHECK(read.linear == opening{}.linear);
}

TEST_CASE("every field written through the declared keys reads back as it was set", "[manipulator][configuration]")
{
    const opening written = every_field_moved();

    stands_at(read_velocity_kinematics(saved_and_reloaded(cleared("written.xml"), write_velocity_kinematics(written, velocity_at)), velocity_at), written);
}

// The window is what a learner moves a setting through, so the edits it offers are what has to reach
// the document; a window offering none saves nothing however well the writer works.
TEST_CASE("a window's own settings reach a document through the edits it offers, and one standing at what that document carries offers none", "[manipulator][configuration]")
{
    velocity_stage headless;
    const config::location at = cleared("window.xml");
    const velocity_kinematics_window panel("Velocity kinematics", headless.source->reader(), headless.arm(), headless.shown, velocity_kinematics_window::controls{}, every_field_moved(),
                                           std::string(velocity_at));

    REQUIRE(panel.as_configurable() != nullptr);
    const std::vector<config::edit> offered = panel.as_configurable()->settings_edits(loaded(at));
    REQUIRE_FALSE(offered.empty());

    const config::document written = saved_and_reloaded(at, offered);
    stands_at(read_velocity_kinematics(written, velocity_at), every_field_moved());

    const velocity_kinematics_window standing("Velocity kinematics", headless.source->reader(), headless.arm(), headless.shown, velocity_kinematics_window::controls{},
                                              read_velocity_kinematics(written, velocity_at), std::string(velocity_at));

    REQUIRE(standing.as_configurable() != nullptr);
    CHECK(standing.as_configurable()->settings_edits(written).empty());
}

TEST_CASE("a document naming nothing shows both ellipsoids, and one naming them hidden hides them", "[manipulator][configuration]")
{
    CHECK(read_velocity_kinematics(carrying("ellipsoids-absent.xml", ""), velocity_at).ellipsoids);
    CHECK_FALSE(read_velocity_kinematics(carrying("ellipsoids-hidden.xml", "<velocity_kinematics ellipsoids=\"false\"/>"), velocity_at).ellipsoids);
}

TEST_CASE("both ellipsoids hidden, written through the declared keys, read back hidden", "[manipulator][configuration]")
{
    opening written{};
    written.ellipsoids = false;

    CHECK_FALSE(read_velocity_kinematics(saved_and_reloaded(cleared("ellipsoids-written.xml"), write_velocity_kinematics(written, velocity_at)), velocity_at).ellipsoids);
}

TEST_CASE("a window opened with both ellipsoids hidden offers what saves them hidden, and one standing at that document offers none", "[manipulator][configuration]")
{
    velocity_stage headless;
    const config::location at = cleared("ellipsoids-window.xml");
    opening hidden{};
    hidden.ellipsoids = false;
    const velocity_kinematics_window panel("Velocity kinematics", headless.source->reader(), headless.arm(), headless.shown, velocity_kinematics_window::controls{}, hidden,
                                           std::string(velocity_at));

    REQUIRE(panel.as_configurable() != nullptr);
    const config::document written = saved_and_reloaded(at, panel.as_configurable()->settings_edits(loaded(at)));
    CHECK_FALSE(read_velocity_kinematics(written, velocity_at).ellipsoids);

    const velocity_kinematics_window standing("Velocity kinematics", headless.source->reader(), headless.arm(), headless.shown, velocity_kinematics_window::controls{},
                                              read_velocity_kinematics(written, velocity_at), std::string(velocity_at));

    REQUIRE(standing.as_configurable() != nullptr);
    CHECK(standing.as_configurable()->settings_edits(written).empty());
}

TEST_CASE("a document naming no highlighted joint opens at none, one counted from one reads it counted from zero, and one below one names none", "[manipulator][configuration]")
{
    CHECK_FALSE(read_velocity_kinematics(carrying("highlighted-absent.xml", ""), velocity_at).highlighted.has_value());
    CHECK(read_velocity_kinematics(carrying("highlighted-two.xml", "<velocity_kinematics highlighted=\"2\"/>"), velocity_at).highlighted == std::optional<std::size_t>(1u));
    CHECK_FALSE(read_velocity_kinematics(carrying("highlighted-zero.xml", "<velocity_kinematics highlighted=\"0\"/>"), velocity_at).highlighted.has_value());
    CHECK_FALSE(read_velocity_kinematics(carrying("highlighted-negative.xml", "<velocity_kinematics highlighted=\"-1\"/>"), velocity_at).highlighted.has_value());
}

TEST_CASE("a highlighted joint and none, written through the declared keys, read back as they were set", "[manipulator][configuration]")
{
    opening picked{};
    picked.highlighted = 1u;

    CHECK(read_velocity_kinematics(saved_and_reloaded(cleared("highlighted-written.xml"), write_velocity_kinematics(picked, velocity_at)), velocity_at).highlighted ==
          picked.highlighted);
    CHECK_FALSE(read_velocity_kinematics(saved_and_reloaded(cleared("highlighted-none.xml"), write_velocity_kinematics(opening{}, velocity_at)), velocity_at).highlighted.has_value());
}

TEST_CASE("a window opened at a highlighted joint offers what saves it, and one standing at that document offers none", "[manipulator][configuration]")
{
    velocity_stage headless;
    const config::location at = cleared("highlighted-window.xml");
    opening picked{};
    picked.highlighted = 1u;
    const velocity_kinematics_window panel("Velocity kinematics", headless.source->reader(), headless.arm(), headless.shown, velocity_kinematics_window::controls{}, picked,
                                           std::string(velocity_at));

    REQUIRE(panel.as_configurable() != nullptr);
    const config::document written = saved_and_reloaded(at, panel.as_configurable()->settings_edits(loaded(at)));
    CHECK(read_velocity_kinematics(written, velocity_at).highlighted == picked.highlighted);

    const velocity_kinematics_window standing("Velocity kinematics", headless.source->reader(), headless.arm(), headless.shown, velocity_kinematics_window::controls{},
                                              read_velocity_kinematics(written, velocity_at), std::string(velocity_at));

    REQUIRE(standing.as_configurable() != nullptr);
    CHECK(standing.as_configurable()->settings_edits(written).empty());
}

TEST_CASE("a window opened from a document naming a joint the arm lacks offers exactly one edit, writing none back", "[manipulator][configuration]")
{
    velocity_stage headless;
    const config::document carried = carrying("highlighted-past.xml", "<velocity_kinematics highlighted=\"9\"/>");
    velocity_kinematics_window panel("Velocity kinematics", headless.source->reader(), headless.arm(), headless.shown, velocity_kinematics_window::controls{},
                                     read_velocity_kinematics(carried, velocity_at), std::string(velocity_at));
    panel.initialize();

    const std::vector<config::edit> offered = panel.settings_edits(carried);
    REQUIRE(offered.size() == 1u);
    CHECK(offered[0].key.ends_with("highlighted"));
    CHECK(offered[0].value == "0");
}

TEST_CASE("a document carrying both rate switches opens a window holding them, tells the stencil, and saves what the window then holds", "[manipulator][configuration]")
{
    const config::location at      = written("rate-switches.xml", "<velocity_kinematics rate_weighted=\"true\" twist=\"false\"/>");
    const config::document carried = loaded(at, config::expectation::partial);
    const opening read             = read_velocity_kinematics(carried, velocity_at);
    CHECK(read.rate_weighted == std::optional<bool>(true));
    CHECK(read.twist == std::optional<bool>(false));

    velocity_stage headless;
    velocity_kinematics_window panel = window_at(headless, read);
    panel.initialize();
    CHECK(headless.shown.columns_weighted());
    CHECK_FALSE(headless.shown.twist_shown());
    CHECK(panel.state().rate_weighted == read.rate_weighted);
    CHECK(panel.state().twist == read.twist);

    opening moved                            = read;
    moved.rate_weighted                      = false;
    moved.twist                              = true;
    const velocity_kinematics_window changed = window_at(headless, moved);

    const opening reread = read_velocity_kinematics(saved_and_reloaded(at, changed.settings_edits(carried)), velocity_at);
    CHECK(reread.rate_weighted == std::optional<bool>(false));
    CHECK(reread.twist == std::optional<bool>(true));
}

TEST_CASE("a document carrying neither rate switch reads neither, offers neither and leaves the stencil where a composition put it", "[manipulator][configuration]")
{
    const config::document carried = carrying("rate-switches-absent.xml", "<velocity_kinematics frame=\"body\"/>");
    opening read                   = read_velocity_kinematics(carried, velocity_at);
    CHECK_FALSE(read.rate_weighted.has_value());
    CHECK_FALSE(read.twist.has_value());

    velocity_stage headless;
    headless.shown.set_columns_weighted(true);
    headless.shown.set_twist_shown(true);
    read.capped                      = false;
    velocity_kinematics_window panel = window_at(headless, read);
    panel.initialize();
    CHECK(headless.shown.columns_weighted());
    CHECK(headless.shown.twist_shown());

    const std::vector<config::edit> offered = panel.settings_edits(carried);
    REQUIRE(offered.size() == 1u);
    CHECK(offered[0].key.ends_with("capped"));
}

TEST_CASE("a document carrying only the weighting reads no twist, and a window moving the weighting saves it and no twist", "[manipulator][configuration]")
{
    const config::location at      = written("rate-weighted-alone.xml", "<velocity_kinematics rate_weighted=\"true\"/>");
    const config::document carried = loaded(at, config::expectation::partial);
    opening read                   = read_velocity_kinematics(carried, velocity_at);
    CHECK(read.rate_weighted == std::optional<bool>(true));
    CHECK_FALSE(read.twist.has_value());

    velocity_stage headless;
    read.rate_weighted                      = false;
    const std::vector<config::edit> offered = window_at(headless, read).settings_edits(carried);
    REQUIRE(offered.size() == 1u);
    CHECK(offered[0].key.ends_with("rate_weighted"));
    CHECK(offered[0].value == "false");

    const opening reread = read_velocity_kinematics(saved_and_reloaded(at, offered), velocity_at);
    CHECK(reread.rate_weighted == std::optional<bool>(false));
    CHECK_FALSE(reread.twist.has_value());
    CHECK(bytes_of(at).find("twist") == std::string::npos);
}

TEST_CASE("a rate switch carried at its declared fallback still opts the document in, and a window holding it off saves it over a document lacking it", "[manipulator][configuration]")
{
    CHECK(read_velocity_kinematics(carrying("twist-off.xml", "<velocity_kinematics twist=\"false\"/>"), velocity_at).twist == std::optional<bool>(false));

    const config::location at      = written("weighting-off.xml", "<velocity_kinematics frame=\"space\"/>");
    const config::document carried = loaded(at, config::expectation::partial);
    opening off{};
    off.rate_weighted = false;

    velocity_stage headless;
    const std::vector<config::edit> offered = window_at(headless, off).settings_edits(carried);
    REQUIRE(offered.size() == 1u);
    CHECK(offered[0].key.ends_with("rate_weighted"));

    const config::document saved = saved_and_reloaded(at, offered);
    const opening reread         = read_velocity_kinematics(saved, velocity_at);
    CHECK(reread.rate_weighted == std::optional<bool>(false));
    CHECK_FALSE(reread.twist.has_value());
    CHECK(window_at(headless, reread).settings_edits(saved).empty());
}

TEST_CASE("a document written from the declaration alone carries both rate switches off", "[manipulator][configuration]")
{
    const opening read = read_velocity_kinematics(loaded(cleared("rate-switches-template.xml")), velocity_at);

    CHECK(read.rate_weighted == std::optional<bool>(false));
    CHECK(read.twist == std::optional<bool>(false));
}

TEST_CASE("the writer lists the rate switches after every other field, the weighting before the twist, and a window's offer keeps that order", "[manipulator][configuration]")
{
    const std::vector<config::edit> plain = write_velocity_kinematics(opening{}, velocity_at);
    CHECK(plain.size() == 8u);
    CHECK(std::ranges::none_of(plain, names_a_rate_switch));

    opening both                        = every_field_moved();
    both.highlighted                    = 1u;
    both.rate_weighted                  = true;
    both.twist                          = true;
    const std::vector<config::edit> all = write_velocity_kinematics(both, velocity_at);
    REQUIRE(all.size() == 10u);
    CHECK(all[8].key.ends_with("rate_weighted"));
    CHECK(all[9].key.ends_with("twist"));

    std::vector<std::string> moved = keys_of(all);
    std::erase_if(moved, [](const std::string &key) { return key.ends_with("ellipsoids"); });

    velocity_stage headless;
    CHECK(keys_of(window_at(headless, both).settings_edits(loaded(cleared("rate-switches-order.xml")))) == moved);
}

TEST_CASE("a document carrying neither rate switch saved with its frame moved changes that value's bytes and no other", "[manipulator][configuration]")
{
    const config::location at      = written("rate-switches-bytes.xml",
                                             "<velocity_kinematics frame=\"space\" reading=\"velocity\" angular=\"true\" linear=\"true\" "
                                             "columns=\"true\" capped=\"true\" ellipsoids=\"true\" highlighted=\"0\"/>");
    const std::string before       = bytes_of(at);
    const config::document carried = loaded(at, config::expectation::partial);
    opening read                   = read_velocity_kinematics(carried, velocity_at);

    velocity_stage headless;
    CHECK(window_at(headless, read).settings_edits(carried).empty());

    read.frame                              = jacobian_frame::body;
    const std::vector<config::edit> offered = window_at(headless, read).settings_edits(carried);
    REQUIRE(offered.size() == 1u);
    REQUIRE(config::save(described(), at, offered).has_value());

    std::string expected = before;
    expected.replace(expected.find("frame=\"space\""), std::string_view("frame=\"space\"").size(), "frame=\"body\"");
    const std::string after = bytes_of(at);
    CHECK(after == expected);
    CHECK(after.find("rate_weighted") == std::string::npos);
    CHECK(after.find("twist") == std::string::npos);
}
