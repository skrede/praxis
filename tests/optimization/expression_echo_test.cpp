#include "typed/parse.h"
#include "typed/expression.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <numbers>
#include <string_view>

using namespace praxis::optimization;

namespace {

constexpr double x = 0.7;
constexpr double y = 1.3;

constexpr double p(char letter)
{
    return 0.1 * static_cast<double>(letter - 'a' + 1);
}

constexpr double pi = std::numbers::pi;
constexpr double e  = std::numbers::e;

// The value each row's echo states, written out by hand at x, y and each parameter at p(letter).
struct reading
{
    std::string_view text;
    std::size_t dimension;
    std::string_view echo;
    std::string_view parameters;
    double (*value)();
};

const std::array readings{
        reading{"xb", 1, "x*b", "b", [] { return x * p('b'); }},
        reading{"1/2x", 1, "(1/2)*x", "", [] { return (1.0 / 2.0) * x; }},
        reading{"2x^3", 1, "2*x^3", "", [] { return 2.0 * std::pow(x, 3.0); }},
        reading{"cos(2dx)", 1, "cos(2*d*x)", "d", [] { return std::cos(2.0 * p('d') * x); }},
        reading{"tan(x)", 1, "tan(x)", "", [] { return std::tan(x); }},
        reading{"-x^2", 1, "-(x^2)", "", [] { return -std::pow(x, 2.0); }},
        reading{"2^3^2", 1, "2^(3^2)", "", [] { return std::pow(2.0, std::pow(3.0, 2.0)); }},
        reading{"e^-x", 1, "e^(-x)", "", [] { return std::pow(e, -x); }},
        reading{"2^-x^2", 1, "2^(-(x^2))", "", [] { return std::pow(2.0, -std::pow(x, 2.0)); }},
        reading{"xy", 2, "x*y", "", [] { return x * y; }},
        reading{"xy", 1, "x*y", "y", [] { return x * p('y'); }},
        reading{"spin", 1, "s*pi*n", "ns", [] { return p('s') * pi * p('n'); }},
        reading{"texp(x)", 1, "t*exp(x)", "t", [] { return p('t') * std::exp(x); }},
        reading{"pie", 1, "pi*e", "", [] { return pi * e; }},
        reading{"ex", 1, "e*x", "", [] { return e * x; }},
        reading{"2e3", 1, "2*e*3", "", [] { return 2.0 * e * 3.0; }},
        reading{"x/2/3", 1, "(x/2)/3", "", [] { return (x / 2.0) / 3.0; }},
        reading{"x/(2y)", 2, "x/(2*y)", "", [] { return x / (2.0 * y); }},
        reading{"a - (b - c)", 1, "a - (b - c)", "abc", [] { return p('a') - (p('b') - p('c')); }},
        reading{"x - -2", 1, "x - (-2)", "", [] { return x - (-2.0); }},
        reading{"x + -2", 1, "x + -2", "", [] { return x + -2.0; }},
        reading{"--x", 1, "-(-x)", "", [] { return -(-x); }},
        reading{"-x*y", 2, "(-x)*y", "", [] { return (-x) * y; }},
        reading{"2(x + 1)", 1, "2*(x + 1)", "", [] { return 2.0 * (x + 1.0); }},
        reading{"(x + 1)(x - 1)", 1, "(x + 1)*(x - 1)", "", [] { return (x + 1.0) * (x - 1.0); }},
        reading{"sin(x)cos(x)", 1, "sin(x)*cos(x)", "", [] { return std::sin(x) * std::cos(x); }},
        reading{"x2", 1, "x*2", "", [] { return x * 2.0; }},
        reading{".5x", 1, ".5*x", "", [] { return 0.5 * x; }},
        reading{"0.1x", 1, "0.1*x", "", [] { return 0.1 * x; }},
        reading{"x^(1/2)", 1, "x^(1/2)", "", [] { return std::pow(x, 1.0 / 2.0); }},
        reading{"(-x)^2", 1, "(-x)^2", "", [] { return std::pow(-x, 2.0); }},
        reading{"sqrt(x^2 + y^2)", 2, "sqrt(x^2 + y^2)", "", [] { return std::sqrt(std::pow(x, 2.0) + std::pow(y, 2.0)); }},
        reading{"log(x)", 1, "log(x)", "", [] { return std::log(x); }},
        reading{"  2 x ", 1, "2*x", "", [] { return 2.0 * x; }},
};

double applied(typed::named_function function, double u)
{
    constexpr std::array<double (*)(double), 6> functions{
            [](double v) { return std::sin(v); }, [](double v) { return std::cos(v); }, [](double v) { return std::tan(v); },
            [](double v) { return std::exp(v); }, [](double v) { return std::log(v); }, [](double v) { return std::sqrt(v); },
    };
    return functions[static_cast<std::size_t>(function)](u);
}

double combined(const typed::node &one, double a, double b)
{
    switch(one.kind)
    {
        case typed::node_kind::add:
            return a + b;
        case typed::node_kind::subtract:
            return a - b;
        case typed::node_kind::multiply:
            return a * b;
        case typed::node_kind::divide:
            return a / b;
        default:
            return std::pow(a, b);
    }
}

double value_of(const typed::node &one, const std::vector<double> &below)
{
    switch(one.kind)
    {
        case typed::node_kind::number:
        case typed::node_kind::constant:
            return one.value;
        case typed::node_kind::variable:
            return one.letter == 'x' ? x : y;
        case typed::node_kind::parameter:
            return p(one.letter);
        case typed::node_kind::negate:
            return -below[one.left];
        case typed::node_kind::function:
            return applied(one.function, below[one.left]);
        default:
            return combined(one, below[one.left], below[one.right]);
    }
}

double evaluated(const typed::expression &read)
{
    std::vector<double> below;
    for(const typed::node &one : read.nodes)
        below.push_back(value_of(one, below));
    return below.back();
}

}

TEST_CASE("a typed function echoes with explicit operators and names its parameters")
{
    const auto read = typed::parsed("2x^3 + 5b", 1);

    REQUIRE(read.has_value());
    CHECK(typed::echoed(*read) == "2*x^3 + 5*b");
    CHECK(read->parameters == "b");
}

TEST_CASE("every reading of the grammar echoes as it was read")
{
    for(const reading &row : readings)
    {
        CAPTURE(row.text, row.dimension);
        const auto read = typed::parsed(row.text, row.dimension);
        REQUIRE(read.has_value());
        CHECK(typed::echoed(*read) == row.echo);
        CHECK(read->parameters == row.parameters);
    }
}

TEST_CASE("every reading's tree has the value its echo states")
{
    for(const reading &row : readings)
    {
        CAPTURE(row.text, row.dimension);
        const auto read = typed::parsed(row.text, row.dimension);
        REQUIRE(read.has_value());
        CHECK(evaluated(*read) == Catch::Approx(row.value()).epsilon(1.0e-12));
    }
}
