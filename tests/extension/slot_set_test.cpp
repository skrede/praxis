#include "praxis/extension.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>

using namespace praxis;

namespace {

enum class pair_slot : std::uint32_t
{
    first,
    second,
    count
};

using pair_slot_set = basic_slot_set<pair_slot>;

std::uint32_t slot_count()
{
    return static_cast<std::uint32_t>(pair_slot::count);
}

}

static_assert(pair_slot_set().empty());
static_assert(pair_slot_set().set(pair_slot::first).contains(pair_slot::first));

TEST_CASE("a_default_constructed_slot_set_is_empty_and_contains_nothing")
{
    pair_slot_set expected;

    REQUIRE(expected.empty());
    REQUIRE_FALSE(expected.contains(pair_slot::first));
    REQUIRE_FALSE(expected.contains(pair_slot::second));
}

TEST_CASE("the_count_enumerator_names_no_slot_and_enters_no_set")
{
    pair_slot_set expected;

    REQUIRE(expected.set(pair_slot::count).empty());
    REQUIRE_FALSE(expected.contains(pair_slot::count));
}

TEST_CASE("slot_sets_compose_by_union_intersection_and_complement")
{
    pair_slot_set one;
    pair_slot_set two;

    one.set(pair_slot::first);
    two.set(pair_slot::second);

    pair_slot_set both   = one | two;
    pair_slot_set shared = one & two;

    REQUIRE(both.contains(pair_slot::first));
    REQUIRE(both.contains(pair_slot::second));
    REQUIRE(shared.empty());
    REQUIRE((one & ~one).empty());
}

TEST_CASE("complementing_moves_between_the_empty_set_and_the_set_of_every_slot")
{
    pair_slot_set every;
    pair_slot_set complement_of_none = ~pair_slot_set();
    std::uint32_t held               = 0;

    for(std::uint32_t index = 0; index < slot_count(); ++index)
    {
        every.set(static_cast<pair_slot>(index));
        held += complement_of_none.contains(static_cast<pair_slot>(index)) ? 1u : 0u;
    }

    REQUIRE(held == slot_count());
    REQUIRE_FALSE(every.empty());
    REQUIRE((~every).empty());
}
