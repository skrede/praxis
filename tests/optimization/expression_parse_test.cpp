#include "typed/parse.h"
#include "typed/expression.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <limits>
#include <string>
#include <cstddef>
#include <string_view>

using namespace praxis::optimization;

namespace {

struct refusal
{
    std::string_view text;
    std::size_t offset;
    std::string_view expected;
};

constexpr std::string_view operand = "an operand";
constexpr std::string_view unread  = "a digit, a letter, an operator or a parenthesis";
constexpr std::string_view numbers = "an operator between two numbers";

const std::array refusals{
        refusal{"", 0, operand},
        refusal{"x +", 3, operand},
        refusal{"x^", 2, operand},
        refusal{"*x", 0, operand},
        refusal{"+x", 0, operand},
        refusal{"(x", 2, "')'"},
        refusal{"sqrt(x", 6, "')'"},
        refusal{"x)", 1, "an operator or the end"},
        refusal{"sinx", 3, "'(' after sin"},
        refusal{"sinh(x)", 3, "'(' after sin"},
        refusal{"exp", 3, "'(' after exp"},
        refusal{"2 3", 2, numbers},
        refusal{"1.5.2", 3, numbers},
        refusal{"x^2 3", 4, numbers},
        refusal{".", 0, "a digit"},
        refusal{"X", 0, "a lowercase letter"},
        refusal{"2X", 1, "a lowercase letter"},
        refusal{"x $", 2, unread},
        refusal{"x\xC3\xA9", 1, unread},
        refusal{"x\tx", 1, unread},
};

}

TEST_CASE("every refusal of the grammar names its offset and what was expected there")
{
    for(const refusal &row : refusals)
    {
        CAPTURE(row.text);
        const auto read = typed::parsed(row.text, 1);
        REQUIRE_FALSE(read.has_value());
        CHECK(read.error().offset == row.offset);
        CHECK(read.error().expected == row.expected);
    }
}

TEST_CASE("a number as long as the capacity is read where a double holds it and refused at its first digit where not")
{
    const auto read = typed::parsed(std::string(typed::expression_capacity, '9'), 1);

    if(typed::expression_capacity <= std::numeric_limits<double>::max_exponent10)
    {
        CHECK(read.has_value());
        return;
    }
    REQUIRE_FALSE(read.has_value());
    CHECK(read.error().offset == 0);
    CHECK(read.error().expected == "a number a double holds");
}

TEST_CASE("a text filling the capacity with nested parentheses or with unary minuses is read")
{
    const std::size_t depth = (typed::expression_capacity - 2) / 2;
    const auto nested       = typed::parsed(std::string(depth, '(') + "x" + std::string(depth, ')'), 1);
    const auto negated      = typed::parsed(std::string(typed::expression_capacity - 2, '-') + "x", 1);

    REQUIRE(nested.has_value());
    CHECK(nested->nodes.size() == 1);
    REQUIRE(negated.has_value());
    CHECK(negated->nodes.size() == typed::expression_capacity - 1);
}

TEST_CASE("a text as long as the capacity is read and a longer one is refused at the capacity's offset")
{
    CHECK(typed::parsed(std::string(typed::expression_capacity, 'x'), 1).has_value());

    const auto read = typed::parsed(std::string(typed::expression_capacity + 1, 'x'), 1);
    REQUIRE_FALSE(read.has_value());
    CHECK(read.error().offset == typed::expression_capacity);
}
