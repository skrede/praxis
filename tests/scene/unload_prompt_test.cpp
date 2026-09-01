#include "held_composition.h"

#include "praxis/scene/composition.h"

#include <catch2/catch_test_macros.hpp>

#include <cstddef>

using namespace praxis;
using namespace praxis::scene;
using namespace praxis::fixture;

TEST_CASE("a composition with nothing to decide releases where it is asked to", "[scene][preset]")
{
    held_scene over(false);

    SECTION("with a question answering that there is nothing to decide")
    {
        over.asks();
    }

    SECTION("with no question installed at all")
    {
    }

    const std::size_t before = over.counted();

    REQUIRE(over.held.load(composing()).has_value());
    REQUIRE(over.counted() == before + 1);

    over.held.unload();

    REQUIRE(over.watched.resolved == 0);
    REQUIRE_FALSE(over.held.loaded());
    REQUIRE_FALSE(over.held.awaiting_answer());
    REQUIRE(over.counted() == before);
}

TEST_CASE("a composition with a question outstanding releases nothing until it is answered", "[scene][preset]")
{
    held_scene over(true);

    over.asks();

    const std::size_t before = over.counted();

    REQUIRE(over.held.load(composing()).has_value());

    const std::size_t composed = over.counted();

    over.held.unload();

    REQUIRE(over.watched.asked == 1);
    REQUIRE(over.watched.resolved == 0);
    REQUIRE(over.held.awaiting_answer());
    REQUIRE(over.held.loaded());
    REQUIRE(over.held.composed() != nullptr);
    REQUIRE(over.counted() == composed);

    leaving_answer chosen{true, true};

    SECTION("keeping")
    {
    }

    SECTION("discarding")
    {
        chosen = leaving_answer{false, false};
    }

    over.held.answer(chosen);

    REQUIRE(over.watched.resolved == 1);
    REQUIRE(over.watched.given.keep == chosen.keep);
    REQUIRE(over.watched.given.remember == chosen.remember);
    REQUIRE_FALSE(over.held.awaiting_answer());
    REQUIRE_FALSE(over.held.loaded());
    REQUIRE(over.counted() == before);

    over.held.answer(leaving_answer{!chosen.keep, false});

    REQUIRE(over.watched.resolved == 1);
    REQUIRE(over.watched.given.keep == chosen.keep);
}

TEST_CASE("an answer to a composition awaiting none does nothing at all", "[scene][preset]")
{
    held_scene over(true);

    over.asks();

    REQUIRE(over.held.load(composing()).has_value());

    const std::size_t composed = over.counted();

    over.held.answer(leaving_answer{false, false});

    REQUIRE(over.watched.asked == 0);
    REQUIRE(over.watched.resolved == 0);
    REQUIRE(over.held.loaded());
    REQUIRE(over.counted() == composed);
}
