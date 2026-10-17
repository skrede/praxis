#include "praxis/extension/refusal.h"

#include <catch2/catch_test_macros.hpp>

using namespace praxis;

static_assert(refusal_text(refusal::degenerate) == "degenerate");

TEST_CASE("each_refusal_is_named_by_the_words_its_reports_spell")
{
    REQUIRE(refusal_text(refusal::unsupported_input) == "unsupported input");
    REQUIRE(refusal_text(refusal::degenerate) == "degenerate");
    REQUIRE(refusal_text(refusal::no_solution) == "no solution");
    REQUIRE(refusal_text(refusal::not_implemented) == "not implemented");
}

TEST_CASE("a_value_outside_the_enumeration_is_named_unclassified")
{
    REQUIRE(refusal_text(static_cast<refusal>(4)) == "unclassified");
}
