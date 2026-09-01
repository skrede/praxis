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
#include <fstream>
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
    CHECK(read.angular_ellipsoid == written.angular_ellipsoid);
    CHECK(read.linear_ellipsoid == written.linear_ellipsoid);
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
    const config::document carried = carrying("authored.xml",
                                              "<velocity_kinematics frame=\"body\" reading=\"force\" angular_ellipsoid=\"false\" linear_ellipsoid=\"false\" columns=\"false\" "
                                              "capped=\"false\"/>");

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
