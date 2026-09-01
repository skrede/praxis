#include "praxis/extension.h"

#include <catch2/catch_test_macros.hpp>

#include <set>
#include <version>
#include <cstdint>
#include <type_traits>

#if defined(__cpp_lib_expected)
    #include <expected>
#endif

using namespace praxis;

namespace {

expected<int, refusal> engaged()
{
    return 4;
}

expected<int, refusal> disengaged()
{
    return praxis::unexpected(refusal::degenerate);
}

int plain()
{
    return 4;
}

}

TEST_CASE("an engaged result yields its value and a disengaged one yields its enumerator")
{
    const auto value = engaged();
    REQUIRE(value.has_value());
    REQUIRE(*value == 4);

    const auto error = disengaged();
    REQUIRE_FALSE(error.has_value());
    REQUIRE(error.error() == refusal::degenerate);
}

TEST_CASE("value_or yields the held value when engaged and the argument when not")
{
    REQUIRE(engaged().value_or(9) == 4);
    REQUIRE(disengaged().value_or(9) == 9);
}

TEST_CASE("a cross-state assignment leaves the target in the assigned state with the assigned payload")
{
    expected<int, refusal> target = engaged();
    target                        = disengaged();
    REQUIRE_FALSE(target.has_value());
    REQUIRE(target.error() == refusal::degenerate);

    target = engaged();
    REQUIRE(target.has_value());
    REQUIRE(*target == 4);

    expected<int, refusal> source = praxis::unexpected(refusal::no_solution);
    target                        = source;
    REQUIRE_FALSE(target.has_value());
    REQUIRE(target.error() == refusal::no_solution);
}

// unexpected is written with no explicit template argument throughout: alias-template class template
// argument deduction (P1814) is GCC-only, so this spelling is what a Clang toolchain rejects if the
// wrapper is ever demoted to an alias template.
TEST_CASE("the error wrapper deduces its type with no explicit template argument")
{
    const expected<int, refusal> deduced = praxis::unexpected(refusal::no_solution);
    REQUIRE_FALSE(deduced.has_value());
    REQUIRE(deduced.error() == refusal::no_solution);

    STATIC_REQUIRE(std::is_same_v<decltype(praxis::unexpected(refusal::degenerate)), praxis::unexpected<refusal>>);
}

TEST_CASE("the valueless specialization default-constructs engaged and converts from the error wrapper")
{
    const expected<void, refusal> done;
    REQUIRE(done.has_value());

    const expected<void, refusal> refused = praxis::unexpected(refusal::not_implemented);
    REQUIRE_FALSE(refused.has_value());
    REQUIRE(refused.error() == refusal::not_implemented);

    expected<void, refusal> target = refused;
    target                         = done;
    REQUIRE(target.has_value());
}

TEST_CASE("the four enumerators are distinct and the vocabulary is one byte wide")
{
    const std::set<refusal> all{refusal::unsupported_input, refusal::degenerate, refusal::no_solution, refusal::not_implemented};
    REQUIRE(all.size() == 4u);

    STATIC_REQUIRE(std::is_same_v<std::underlying_type_t<refusal>, std::uint8_t>);
}

TEST_CASE("a function pointer is reported as carrying the refusal channel only when its return type does")
{
    STATIC_REQUIRE(returns_refusal_v<decltype(&engaged)>);
    STATIC_REQUIRE_FALSE(returns_refusal_v<decltype(&plain)>);
    STATIC_REQUIRE(returns_refusal_v<expected<void, refusal> (*)()>);
    STATIC_REQUIRE_FALSE(returns_refusal_v<expected<int, int> (*)()>);
}

#if defined(__cpp_lib_expected)
TEST_CASE("the boundary converters round-trip a value and an error through the standard type")
{
    const auto value = static_cast<std::expected<int, refusal>>(engaged());
    REQUIRE(value.has_value());
    REQUIRE(*value == 4);
    REQUIRE(*expected<int, refusal>(value) == 4);

    const auto error = static_cast<std::expected<int, refusal>>(disengaged());
    REQUIRE_FALSE(error.has_value());
    REQUIRE(error.error() == refusal::degenerate);
    REQUIRE(expected<int, refusal>(error).error() == refusal::degenerate);

    const auto wrapped = static_cast<std::unexpected<refusal>>(praxis::unexpected(refusal::no_solution));
    REQUIRE(wrapped.error() == refusal::no_solution);
    REQUIRE(praxis::unexpected<refusal>(wrapped).error() == refusal::no_solution);
}
#endif
