#include "praxis/compat/detail/callable.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <memory>
#include <utility>
#include <version>
#include <stdexcept>
#include <type_traits>

using namespace praxis;

namespace {

using nullary = detail::move_only_function<void()>;

// Larger than any small-buffer budget a port of this shape would choose, so the spilled path is
// exercised whichever value the buffer carries.
struct oversized
{
    std::array<double, 512> filler;
    int *counter;

    void operator()() const
    {
        ++*counter;
    }
};

}

TEST_CASE("a callable is move-only", "[callable]")
{
    STATIC_REQUIRE(std::is_move_constructible_v<nullary>);
    STATIC_REQUIRE(std::is_move_assignable_v<nullary>);
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<nullary>);
    STATIC_REQUIRE_FALSE(std::is_copy_assignable_v<nullary>);
}

TEST_CASE("a default-constructed callable holds no target", "[callable]")
{
    const nullary empty;

    REQUIRE_FALSE(static_cast<bool>(empty));
    REQUIRE(empty == nullptr);
}

TEST_CASE("a callable holds a move-only target", "[callable]")
{
    int ran      = 0;
    auto owned   = std::make_unique<int>(7);
    nullary work = [&ran, held = std::move(owned)] { ran += *held; };

    REQUIRE(static_cast<bool>(work));
    work();

    REQUIRE(ran == 7);
}

TEST_CASE("a callable holds a target larger than its buffer", "[callable]")
{
    int ran      = 0;
    nullary work = oversized{{}, &ran};

    work();
    work();

    REQUIRE(ran == 2);
}

TEST_CASE("moving a callable leaves the source holding no target", "[callable]")
{
    int ran      = 0;
    nullary work = [&ran] { ++ran; };

    nullary moved = std::move(work);
    moved();

    REQUIRE(ran == 1);
    REQUIRE(work == nullptr);
}

TEST_CASE("move assignment releases whatever the target was", "[callable]")
{
    int first           = 0;
    int second          = 0;
    nullary work        = [&first] { ++first; };
    nullary replacement = [&second] { ++second; };

    work = std::move(replacement);
    work();

    REQUIRE(first == 0);
    REQUIRE(second == 1);
}

TEST_CASE("a callable carries arguments and a return", "[callable]")
{
    detail::move_only_function<int(int, int)> sum = [](int left, int right) { return left + right; };

    REQUIRE(sum(2, 3) == 5);
}

TEST_CASE("a callable destroys its target exactly once", "[callable]")
{
    const auto witness = std::make_shared<int>(0);

    {
        nullary work = [held = witness] { *held += 1; };
        work();
        REQUIRE(witness.use_count() == 2);
    }

    REQUIRE(*witness == 1);
    REQUIRE(witness.use_count() == 1);
}

#if !defined(__cpp_lib_move_only_function)
TEST_CASE("calling a callable with no target is reported rather than run", "[callable]")
{
    nullary empty;

    REQUIRE_THROWS_AS(empty(), std::logic_error);
}
#endif
