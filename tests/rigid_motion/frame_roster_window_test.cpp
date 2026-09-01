#include "frame_roster.h"

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/frame_roster_window.h"

#include "praxis/extension/refusal.h"

#include <catch2/catch_test_macros.hpp>

#include <set>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::rigid_motion;

namespace {

constexpr std::size_t opened_with = 3;

}

TEST_CASE("the create control adds a frame to the stencil the roster drives and stands on it", "[rigid_motion][windows]")
{
    rostered staged;
    REQUIRE(staged.body.count() == opened_with);
    REQUIRE(staged.standing == 0u);

    press_on(staged.roster, "Create");

    REQUIRE(staged.body.count() == opened_with + 1);
    REQUIRE_FALSE(staged.body.name_of(opened_with).empty());
    REQUIRE(staged.standing == opened_with);
}

TEST_CASE("a name typed into the field is the name the created frame carries", "[rigid_motion][windows]")
{
    rostered staged;

    type_and_create(staged, "Name", "Wrist");

    REQUIRE(staged.body.count() == opened_with + 1);
    REQUIRE(staged.body.name_of(opened_with) == "Wrist");
}

TEST_CASE("creating with no name given names the frame, and never twice the same", "[rigid_motion][windows]")
{
    rostered staged;

    std::set<std::string> handed;
    for(std::size_t made = 0; made < 10; ++made)
    {
        const std::string named = created_by(staged);

        REQUIRE_FALSE(named.empty());
        REQUIRE(handed.insert(named).second);
    }

    REQUIRE(staged.body.count() == opened_with + 10);
    for(const std::string &named : names_of(staged.body))
        REQUIRE_FALSE(named.empty());
}

// A name is the key an arrangement document is written under, so a name a removed frame carried
// must not be handed to a later frame: the document already carries a placement under it.
TEST_CASE("a name a removed frame carried is not given to a later one", "[rigid_motion][windows]")
{
    rostered staged;

    const std::string first = created_by(staged);

    REQUIRE(staged.roster.remove_selected().has_value());
    REQUIRE(staged.body.count() == opened_with);

    const std::string second = created_by(staged);

    REQUIRE(second != first);
}

TEST_CASE("a name another frame already carries is refused and nothing is created", "[rigid_motion][windows]")
{
    rostered staged;
    const std::size_t drawn = geometry_of(staged.roster);

    const expected<std::size_t, refusal> made = staged.roster.create("One");

    REQUIRE_FALSE(made.has_value());
    REQUIRE(made.error() == refusal::unsupported_input);
    REQUIRE(staged.body.count() == opened_with);
    REQUIRE(geometry_of(staged.roster) != drawn);
}

TEST_CASE("the name the fixed frame carries is refused and nothing is created", "[rigid_motion][windows]")
{
    rostered staged("Ground");
    const std::size_t drawn = geometry_of(staged.roster);

    const expected<std::size_t, refusal> made = staged.roster.create("Ground");

    REQUIRE_FALSE(made.has_value());
    REQUIRE(made.error() == refusal::unsupported_input);
    REQUIRE(staged.body.count() == opened_with);
    REQUIRE(geometry_of(staged.roster) != drawn);
}

// The fixed frame carries the name the generator would otherwise hand out first.
TEST_CASE("a generated name never collides with the fixed frame's", "[rigid_motion][windows]")
{
    const std::string taken = std::string(rostered_stem) + " 1";
    rostered staged(taken);

    const std::string named = created_by(staged);

    REQUIRE(named != taken);
    REQUIRE(staged.body.count() == opened_with + 1);
    for(const std::string &carried : names_of(staged.body))
        REQUIRE(carried != taken);
}

TEST_CASE("a generated name is built from the noun the composition handed the roster", "[rigid_motion][windows]")
{
    rostered staged;

    const std::string named = created_by(staged);

    REQUIRE(named.starts_with(rostered_stem));
    REQUIRE(named == std::string(rostered_stem) + " 1");
}

// Two rosters given two nouns hand out two different names from the same ordinal, which no noun the
// library could hold would satisfy: one of the two halves would always read the library's instead.
TEST_CASE("two rosters given two nouns generate two differently named frames", "[rigid_motion][windows]")
{
    rostered here(std::string(), "Rail");
    rostered there(std::string(), "Rung");

    const std::string first  = created_by(here);
    const std::string second = created_by(there);

    REQUIRE(first == "Rail 1");
    REQUIRE(second == "Rung 1");
    REQUIRE(first != second);
}

TEST_CASE("the remove control takes the frame the roster stands on out of the stencil", "[rigid_motion][windows]")
{
    rostered staged;

    press_below_top(staged.roster, row_step(2));

    REQUIRE(staged.standing == 2u);

    press_on(staged.roster, "Remove");

    REQUIRE(staged.body.count() == opened_with - 1);
    REQUIRE(names_of(staged.body) == std::vector<std::string>{"Fixed", "One"});
}

TEST_CASE("a frame another frame is expressed in is not removed, and the refusal is shown", "[rigid_motion][windows]")
{
    rostered staged;
    REQUIRE(staged.body.set_parent(2, 1).has_value());

    press_below_top(staged.roster, row_step(1));
    const std::size_t drawn = geometry_of(staged.roster);

    const expected<void, refusal> accepted = staged.roster.remove_selected();

    REQUIRE_FALSE(accepted.has_value());
    REQUIRE(accepted.error() == refusal::no_solution);
    REQUIRE(staged.body.count() == opened_with);
    REQUIRE(geometry_of(staged.roster) != drawn);
}

TEST_CASE("a refused removal leaves the selection where it was and an accepted one names a frame that exists", "[rigid_motion][windows]")
{
    rostered staged;
    REQUIRE(staged.body.set_parent(2, 1).has_value());

    press_below_top(staged.roster, row_step(1));
    REQUIRE_FALSE(staged.roster.remove_selected().has_value());
    REQUIRE(staged.standing == 1u);

    REQUIRE(staged.body.set_parent(2, std::nullopt).has_value());

    press_below_top(staged.roster, row_step(2));
    REQUIRE(staged.standing == 2u);
    REQUIRE(staged.roster.remove_selected().has_value());
    REQUIRE(staged.standing < staged.body.count());
    REQUIRE_FALSE(staged.body.name_of(staged.standing).empty());
}

// Nothing is asked of the window between the two drawings: the frame set lives in the stencil and is
// read at every draw, so an add made anywhere reaches the list on its own.
TEST_CASE("the drawn frame list follows an add made directly on the stencil", "[rigid_motion][windows]")
{
    rostered staged;
    const std::size_t before = geometry_of(staged.roster);

    staged.body.add(stencil_object{"Elbow", axes_settings{}, object_body{}});

    REQUIRE(geometry_of(staged.roster) != before);
    REQUIRE(names_of(staged.body).back() == "Elbow");
}

TEST_CASE("the controls that make a frame are drawn above the one that takes it away", "[rigid_motion][windows]")
{
    rostered staged;

    walk_onto_each(staged.roster, {"Name", "Create", "Remove"});
}

// A row press is how the roster says which frame is meant, and there is one selection for it to say
// it into: what a case reads back is the value the composition holds rather than anything the window
// kept for itself.
TEST_CASE("pressing a row moves the one selection the roster is composed over", "[rigid_motion][windows]")
{
    rostered staged;
    REQUIRE(staged.standing == 0u);

    press_below_top(staged.roster, row_step(2));

    REQUIRE(staged.standing == 2u);

    press_below_top(staged.roster, row_step(1));

    REQUIRE(staged.standing == 1u);
}
