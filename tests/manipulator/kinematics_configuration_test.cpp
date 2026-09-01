#include "captured_log.h"

#include "praxis/manipulator/kinematics_configuration.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::manipulator;
using Catch::Matchers::ContainsSubstring;

namespace {

constexpr std::size_t joints = 2u;

constexpr std::string_view seeds_at       = "machine/ik_seeds";
constexpr std::string_view branch_at      = "machine/ik_branch";
constexpr std::string_view iterates_at    = "machine/ik_iterates";
constexpr std::string_view convergence_at = "machine/ik_convergence";

// Both ends carry a start in full double precision, so a list written and read back names the same
// configurations rather than ones near them.
constexpr double exactly = 0.0;

config::declaration described()
{
    config::declaration shape("probe");
    shape.group("machine");
    declare_ik_seeds(shape, seeds_at);
    declare_ik_branch(shape, branch_at);
    declare_ik_iterates(shape, iterates_at);
    declare_ik_convergence(shape, convergence_at);

    return shape;
}

std::filesystem::path scratch()
{
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "praxis-kinematics-configuration";
    std::filesystem::create_directories(directory);

    return directory;
}

config::document nothing_carried(const std::string &name)
{
    const std::filesystem::path where = scratch() / name;
    std::filesystem::remove(where);

    return config::load_or_defaults(described(), config::resolve(where, scratch()), config::expectation::partial).values;
}

config::location cleared(const std::string &name)
{
    const std::filesystem::path where = scratch() / name;
    std::filesystem::remove(where);
    REQUIRE(config::write_template(described(), where).has_value());

    return config::resolve(where, scratch());
}

config::document loaded(const config::location &at)
{
    const config::outcome answered = config::load_or_defaults(described(), at);
    INFO((answered.failure ? answered.failure->message : std::string()));
    REQUIRE_FALSE(answered.failure.has_value());

    return answered.values;
}

config::document saved_and_reloaded(const config::location &at, const std::vector<config::edit> &changes)
{
    const expected<void, config::error> saved = config::save(described(), at, changes);
    INFO((saved ? std::string() : saved.error().message));
    REQUIRE(saved.has_value());

    return loaded(at);
}

config::document carrying(const std::string &name, std::string_view body)
{
    const std::filesystem::path where = scratch() / name;
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << "<probe><machine>" << body << "</machine></probe>\n";
    out.close();

    const config::outcome answered = config::load_or_defaults(described(), config::resolve(where, scratch()), config::expectation::partial);
    INFO((answered.failure ? answered.failure->message : std::string()));
    REQUIRE_FALSE(answered.failure.has_value());

    return answered.values;
}

joint_vector start_at(double first, double second)
{
    joint_vector seed(2);
    seed << first, second;

    return seed;
}

ik_seed_window::settings three_starts()
{
    return ik_seed_window::settings{{start_at(0.25, -0.5), start_at(1.5, 0.125), start_at(-2.25, 3.0)}};
}

// Every key one group declares beneath the one it hangs under, so what two groups have in common is
// read off the declarations themselves rather than off the paths a caller happened to choose.
std::vector<std::string> declared_by(void (*declaring)(config::declaration &, std::string_view), std::string_view at)
{
    config::declaration shape("probe");
    shape.group("machine");
    declaring(shape, at);

    std::vector<std::string> keys;
    for(const config::node &named : shape.nodes())
        if(named.path != "machine")
            keys.push_back(named.path);

    return keys;
}

void stands_at(const joint_vector &read, const joint_vector &written)
{
    REQUIRE(read.size() == written.size());
    CHECK(read.isApprox(written, exactly));
}

}

TEST_CASE("a document declaring nothing yields both windows at the values their settings open at", "[manipulator][configuration]")
{
    const config::document carried = nothing_carried("absent.xml");

    const ik_seed_window::settings starts  = read_ik_seeds(carried, seeds_at, joints);
    const ik_branch_window::settings shown = read_ik_branch(carried, branch_at);
    const ik_branch_window::settings was;

    CHECK(starts.seeds.empty());
    CHECK(shown.mode == was.mode);
    CHECK(shown.figures == was.figures);
}

TEST_CASE("a document carrying a list of starts yields them in the order it carries them", "[manipulator][configuration]")
{
    const config::document carried = carrying("authored.xml",
                                              "<ik_seeds><start><index>1</index><joints>0.25 -0.5</joints></start>"
                                              "<start><index>2</index><joints>1.5 0.125</joints></start>"
                                              "<start><index>3</index><joints>-2.25 3</joints></start></ik_seeds>");

    const ik_seed_window::settings starts = read_ik_seeds(carried, seeds_at, joints);

    REQUIRE(starts.seeds.size() == 3u);
    stands_at(starts.seeds[0], start_at(0.25, -0.5));
    stands_at(starts.seeds[1], start_at(1.5, 0.125));
    stands_at(starts.seeds[2], start_at(-2.25, 3.0));
}

TEST_CASE("a list of starts written through the declared keys reads back as it was set", "[manipulator][configuration]")
{
    const config::location at              = cleared("starts.xml");
    const ik_seed_window::settings written = three_starts();
    const config::document carried         = saved_and_reloaded(at, write_ik_seeds(loaded(at), written, seeds_at));

    const ik_seed_window::settings read = read_ik_seeds(carried, seeds_at, joints);

    REQUIRE(read.seeds.size() == written.seeds.size());
    for(std::size_t start = 0; start < read.seeds.size(); ++start)
        stands_at(read.seeds[start], written.seeds[start]);
}

TEST_CASE("every branch list field written through the declared keys reads back as it was set", "[manipulator][configuration]")
{
    const config::location at                = cleared("branch.xml");
    const ik_branch_window::settings written = ik_branch_window::settings{control_mode::preview, false};
    const config::document carried           = saved_and_reloaded(at, write_ik_branch(written, branch_at));

    const ik_branch_window::settings read = read_ik_branch(carried, branch_at);

    CHECK(read.mode == control_mode::preview);
    CHECK(read.figures == false);
}

TEST_CASE("settings equal to what a document already carries offer no edit at all", "[manipulator][configuration]")
{
    const config::location at              = cleared("unmoved.xml");
    const ik_seed_window::settings starts  = three_starts();
    const ik_branch_window::settings shown = ik_branch_window::settings{control_mode::preview, false};

    std::vector<config::edit> both = write_ik_seeds(loaded(at), starts, seeds_at);
    for(const config::edit &change : write_ik_branch(shown, branch_at))
        both.push_back(change);

    const config::document carried = saved_and_reloaded(at, both);

    CHECK(config::unsaved_edits(carried, write_ik_seeds(carried, starts, seeds_at)).empty());
    CHECK(config::unsaved_edits(carried, write_ik_branch(shown, branch_at)).empty());
}

TEST_CASE("a collection written twice appends once, keyed by the leaf naming the row", "[manipulator][configuration]")
{
    const config::location at             = cleared("twice.xml");
    const ik_seed_window::settings starts = three_starts();

    const config::document once    = saved_and_reloaded(at, write_ik_seeds(loaded(at), starts, seeds_at));
    const config::document carried = saved_and_reloaded(at, write_ik_seeds(once, starts, seeds_at));

    CHECK(carried.identities("machine/ik_seeds/start").size() == 3u);
    CHECK(read_ik_seeds(carried, seeds_at, joints).seeds.size() == 3u);
}

TEST_CASE("a row whose width is not the joint count is declined by name rather than read as a short start", "[manipulator][configuration]")
{
    const config::document carried = carrying("mismatched.xml",
                                              "<ik_seeds><start><index>1</index><joints>0.25 -0.5</joints></start>"
                                              "<start><index>2</index><joints>1.5 0.125 0.5</joints></start></ik_seeds>");

    std::size_t read       = 0;
    const std::string said = reported_by([&carried, &read] { read = read_ik_seeds(carried, seeds_at, joints).seeds.size(); });

    CHECK(read == 1u);
    CHECK_THAT(said, ContainsSubstring("manipulator.read_ik_seeds"));
    CHECK_THAT(said, ContainsSubstring("start of 3 joint values at row 2 for an arm of 2 joints"));
}

TEST_CASE("a row carrying no value at all is a start a shorter list left behind and is passed over", "[manipulator][configuration]")
{
    const config::document carried = carrying("emptied.xml",
                                              "<ik_seeds><start><index>1</index><joints>0.25 -0.5</joints></start>"
                                              "<start><index>2</index><joints></joints></start></ik_seeds>");

    std::size_t read       = 0;
    const std::string said = reported_by([&carried, &read] { read = read_ik_seeds(carried, seeds_at, joints).seeds.size(); });

    CHECK(read == 1u);
    CHECK(said.empty());
}

TEST_CASE("a list saved shorter than the one already carried reads back at its new length", "[manipulator][configuration]")
{
    const config::location at             = cleared("shortened.xml");
    const ik_seed_window::settings starts = three_starts();
    const ik_seed_window::settings fewer  = ik_seed_window::settings{{start_at(0.25, -0.5), start_at(-2.25, 3.0)}};

    const config::document once    = saved_and_reloaded(at, write_ik_seeds(loaded(at), starts, seeds_at));
    const config::document carried = saved_and_reloaded(at, write_ik_seeds(once, fewer, seeds_at));

    const ik_seed_window::settings read = read_ik_seeds(carried, seeds_at, joints);

    REQUIRE(read.seeds.size() == 2u);
    stands_at(read.seeds[0], start_at(0.25, -0.5));
    stands_at(read.seeds[1], start_at(-2.25, 3.0));
}

TEST_CASE("a document declaring nothing yields the iterate table and its plot at the values their settings open at", "[manipulator][configuration]")
{
    const config::document carried = nothing_carried("absent-iterates.xml");

    const ik_iterate_window::settings steps     = read_ik_iterates(carried, iterates_at);
    const ik_convergence_window::settings drawn = read_ik_convergence(carried, convergence_at);
    const ik_iterate_window::settings was;
    const ik_convergence_window::settings drew;

    CHECK(steps.start == was.start);
    CHECK(steps.mode == was.mode);
    CHECK(drawn.angular == drew.angular);
    CHECK(drawn.linear == drew.linear);
}

TEST_CASE("every iterate table field written through the declared keys reads back as it was set", "[manipulator][configuration]")
{
    const config::location at                 = cleared("iterates.xml");
    const ik_iterate_window::settings written = ik_iterate_window::settings{2u, control_mode::preview};
    const config::document carried            = saved_and_reloaded(at, write_ik_iterates(written, iterates_at));

    const ik_iterate_window::settings read = read_ik_iterates(carried, iterates_at);

    CHECK(read.start == 2u);
    CHECK(read.mode == control_mode::preview);
}

TEST_CASE("the document counts the starts from one, and a value naming no row leaves the choice where it stood", "[manipulator][configuration]")
{
    const config::document authored = carrying("counted.xml", "<ik_iterates><start>3</start></ik_iterates>");
    const config::document below    = carrying("uncounted.xml", "<ik_iterates><start>0</start></ik_iterates>");

    CHECK(read_ik_iterates(authored, iterates_at).start == 2u);
    CHECK(read_ik_iterates(below, iterates_at).start == ik_iterate_window::settings{}.start);
}

TEST_CASE("every convergence field written through the declared keys reads back as it was set", "[manipulator][configuration]")
{
    const config::location at                     = cleared("convergence.xml");
    const ik_convergence_window::settings written = ik_convergence_window::settings{false, true};
    const config::document carried                = saved_and_reloaded(at, write_ik_convergence(written, convergence_at));

    const ik_convergence_window::settings read = read_ik_convergence(carried, convergence_at);

    CHECK_FALSE(read.angular);
    CHECK(read.linear);
}

TEST_CASE("settings equal to what a document carries offer no edit for the iterate table or its plot", "[manipulator][configuration]")
{
    const config::location at                   = cleared("unmoved-iterates.xml");
    const ik_iterate_window::settings steps     = ik_iterate_window::settings{1u, control_mode::preview};
    const ik_convergence_window::settings drawn = ik_convergence_window::settings{false, true};

    std::vector<config::edit> both = write_ik_iterates(steps, iterates_at);
    for(const config::edit &change : write_ik_convergence(drawn, convergence_at))
        both.push_back(change);

    const config::document carried = saved_and_reloaded(at, both);

    CHECK(config::unsaved_edits(carried, write_ik_iterates(steps, iterates_at)).empty());
    CHECK(config::unsaved_edits(carried, write_ik_convergence(drawn, convergence_at)).empty());
}

TEST_CASE("the four key groups declare under distinct keys, so none of them reads a key another declares", "[manipulator][configuration]")
{
    const std::vector<std::vector<std::string>> groups{declared_by(&declare_ik_seeds, seeds_at), declared_by(&declare_ik_branch, branch_at),
                                                       declared_by(&declare_ik_iterates, iterates_at), declared_by(&declare_ik_convergence, convergence_at)};

    for(const std::vector<std::string> &group : groups)
        REQUIRE_FALSE(group.empty());

    for(std::size_t one = 0; one < groups.size(); ++one)
        for(std::size_t other = one + 1u; other < groups.size(); ++other)
            for(const std::string &key : groups[one])
                CHECK(std::find(groups[other].begin(), groups[other].end(), key) == groups[other].end());
}
