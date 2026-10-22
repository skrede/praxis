#include "typed/jet.h"
#include "typed/parse.h"
#include "typed/evaluate.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <cstddef>
#include <numbers>
#include <algorithm>
#include <string_view>

using namespace praxis;
using namespace praxis::optimization;

namespace {

struct row
{
    std::string text;
    std::size_t dimension;
    Eigen::Vector2d point;
    double value;
    Eigen::Vector2d gradient;
    Eigen::Matrix2d hessian;
    typed::parameter_values parameters = {};
};

typed::jet jet_of(std::string_view text, std::size_t dimension, const Eigen::Vector2d &point, const typed::parameter_values &parameters = {})
{
    expected<typed::expression, typed::parse_failure> read = typed::parsed(text, dimension);
    REQUIRE(read.has_value());
    return typed::evaluated<typed::jet>(*read, point.head(static_cast<Eigen::Index>(dimension)), parameters);
}

double off_by(double answered, double derived)
{
    return std::abs(answered - derived) / std::max(1.0, std::abs(derived));
}

Eigen::Matrix2d symmetric(double xx, double xy, double yy)
{
    return (Eigen::Matrix2d() << xx, xy, xy, yy).finished();
}

row curve(std::string text, double x, double value, double first, double second)
{
    return row{std::move(text), 1, Eigen::Vector2d(x, 0.0), value, Eigen::Vector2d(first, 0.0), symmetric(second, 0.0, 0.0)};
}

std::vector<row> functions_at(double x)
{
    const double s = std::sin(x), c = std::cos(x), r = std::sqrt(x);
    return {curve("sin(x)", x, s, c, -s),
            curve("cos(x)", x, c, -s, -c),
            curve("tan(x)", x, s / c, 1.0 / (c * c), 2.0 * s / (c * c * c)),
            curve("exp(x)", x, std::exp(x), std::exp(x), std::exp(x)),
            curve("log(x)", x, std::log(x), 1.0 / x, -1.0 / (x * x)),
            curve("sqrt(x)", x, r, 1.0 / (2.0 * r), -1.0 / (4.0 * x * r))};
}

std::vector<row> surfaces_at(double x, double y)
{
    const double c = std::cos(x * y), s = std::sin(x * y);
    return {row{"x^3 y + sin(xy)",
                2,
                {x, y},
                x * x * x * y + s,
                {3.0 * x * x * y + y * c, x * x * x + x * c},
                symmetric(6.0 * x * y - y * y * s, 3.0 * x * x + c - x * y * s, -x * x * s)},
            row{"x/y", 2, {x, y}, x / y, {1.0 / y, -x / (y * y)}, symmetric(0.0, -1.0 / (y * y), 2.0 * x / (y * y * y))},
            row{"-(x*y)", 2, {x, y}, -x * y, {-y, -x}, symmetric(0.0, -1.0, 0.0)}, row{"x^2 - 3y", 2, {x, y}, x * x - 3.0 * y, {2.0 * x, -3.0}, symmetric(2.0, 0.0, 0.0)}};
}

row variable_power(double x, double y)
{
    const double f = std::pow(x, y), l = std::log(x), lowered = std::pow(x, y - 1.0);
    return row{"x^y", 2, {x, y}, f, {y * lowered, f * l}, symmetric(y * (y - 1.0) * std::pow(x, y - 2.0), lowered * (1.0 + y * l), f * l * l)};
}

std::vector<row> table()
{
    std::vector<row> rows = functions_at(0.7);
    for(row &one : surfaces_at(0.7, -1.3))
        rows.push_back(std::move(one));
    rows.push_back(variable_power(1.3, 0.7));

    const double x = 1.5, power = std::pow(x, x), l = std::log(x) + 1.0;
    rows.push_back(curve("x^x", x, power, power * l, power * (l * l + 1.0 / x)));
    rows.push_back(row{"pi x + e y", 2, {0.3, 0.4}, std::numbers::pi * 0.3 + std::numbers::e * 0.4, {std::numbers::pi, std::numbers::e}, Eigen::Matrix2d::Zero()});

    row parameter                   = curve("b x^2", 0.9, 2.5 * 0.81, 2.5 * 1.8, 5.0);
    parameter.parameters['b' - 'a'] = 2.5;
    rows.push_back(parameter);
    return rows;
}

}

TEST_CASE("a typed function's value, gradient and Hessian equal the hand-derived ones within four machine epsilons", "[optimization][typed]")
{
    const double bound = 4.0 * std::numeric_limits<double>::epsilon();
    for(const row &one : table())
    {
        CAPTURE(one.text);
        const typed::jet answered = jet_of(one.text, one.dimension, one.point, one.parameters);
        CHECK(off_by(answered.value, one.value) <= bound);
        for(Eigen::Index i = 0; i < 2; ++i)
        {
            CHECK(off_by(answered.gradient[i], one.gradient[i]) <= bound);
            for(Eigen::Index j = 0; j < 2; ++j)
                CHECK(off_by(answered.hessian(i, j), one.hessian(i, j)) <= bound);
        }
    }
}

TEST_CASE("a power with a constant exponent of zero, one or two at zero answers finite derivatives", "[optimization][typed]")
{
    const Eigen::Vector2d zero = Eigen::Vector2d::Zero();
    const typed::jet none      = jet_of("x^0", 1, zero);
    const typed::jet line      = jet_of("x^1", 1, zero);
    const typed::jet square    = jet_of("x^2", 1, zero);

    CHECK(none.value == 1.0);
    CHECK(none.gradient[0] == 0.0);
    CHECK(none.hessian(0, 0) == 0.0);
    CHECK(line.value == 0.0);
    CHECK(line.gradient[0] == 1.0);
    CHECK(line.hessian(0, 0) == 0.0);
    CHECK(square.value == 0.0);
    CHECK(square.gradient[0] == 0.0);
    CHECK(square.hessian(0, 0) == 2.0);
}
