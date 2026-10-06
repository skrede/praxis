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
#include <optional>
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

config::outcome answering(const std::string &name, std::string_view body)
{
    const std::filesystem::path where = shared_scratch_directory() / name;
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << "<probe><machine>" << body << "</machine></probe>\n";
    out.close();

    return config::load_or_defaults(described(), config::resolve(where, shared_scratch_directory()), config::expectation::partial);
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
