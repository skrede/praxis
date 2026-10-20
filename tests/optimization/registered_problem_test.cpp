#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <functional>

using namespace praxis;
using namespace praxis::optimization;

namespace {

double cubic_value(double x)
{
    return x * x * x - 2.0 * x - 5.0;
}

double cubic_slope(double x)
{
    return 3.0 * x * x - 2.0;
}

Eigen::VectorXd at(double x)
{
    return Eigen::VectorXd::Constant(1, x);
}

}

TEST_CASE("a registered curve's problem answers the book's residual and its slope", "[optimization][registered]")
{
    const problem posed = curve_problem(registered_curve("cubic", &cubic_value, &cubic_slope), 1.0e-3);

    for(const double x : {-4.0, -0.5, 0.0, 2.0, 3.0})
    {
        const expected<Eigen::VectorXd, refusal> e = posed.error(at(x));
        const expected<Eigen::MatrixXd, refusal> j = posed.jacobian(at(x));

        REQUIRE(e);
        REQUIRE(j);
        REQUIRE(e->size() == 1);
        REQUIRE(j->rows() == 1);
        REQUIRE(j->cols() == 1);
        CHECK((*e)[0] == -cubic_value(x));
        CHECK((*j)(0, 0) == cubic_slope(x));
    }
}

TEST_CASE("a registered curve's problem holds converged exactly where the value is inside the tolerance", "[optimization][registered]")
{
    const problem posed = curve_problem(registered_curve("cubic", &cubic_value, &cubic_slope), 0.5);

    CHECK(posed.converged(at(0.0)));
    CHECK(posed.converged(at(0.49)));
    CHECK(posed.converged(at(-0.49)));
    CHECK_FALSE(posed.converged(at(0.5)));
    CHECK_FALSE(posed.converged(at(-0.5)));
    CHECK_FALSE(posed.converged(at(std::numeric_limits<double>::quiet_NaN())));
    CHECK_FALSE(posed.converged(Eigen::VectorXd::Zero(2)));
}

TEST_CASE("a registered curve's problem refuses a value or a slope that is not finite as degenerate", "[optimization][registered]")
{
    const std::function<double(double)> undefined = [](double) { return std::numeric_limits<double>::quiet_NaN(); };
    const std::function<double(double)> vertical  = [](double) { return std::numeric_limits<double>::infinity(); };

    const problem unvalued = curve_problem(registered_curve("undefined", undefined, &cubic_slope), 1.0e-10);
    const problem unsloped = curve_problem(registered_curve("vertical", &cubic_value, vertical), 1.0e-10);

    REQUIRE_FALSE(unvalued.error(at(1.0)));
    CHECK(unvalued.error(at(1.0)).error() == refusal::degenerate);
    CHECK(unvalued.jacobian(at(1.0)));

    REQUIRE_FALSE(unsloped.jacobian(at(1.0)));
    CHECK(unsloped.jacobian(at(1.0)).error() == refusal::degenerate);
    CHECK(unsloped.error(at(1.0)));
}

TEST_CASE("a registered curve's problem refuses a missing function and an x of other than one coordinate as unsupported input", "[optimization][registered]")
{
    const problem unbound = curve_problem(registered_curve("unbound", {}, {}), 1.0e-10);
    const problem posed   = curve_problem(registered_curve("cubic", &cubic_value, &cubic_slope), 1.0e-10);

    CHECK(unbound.error(at(1.0)).error() == refusal::unsupported_input);
    CHECK(unbound.jacobian(at(1.0)).error() == refusal::unsupported_input);
    CHECK(posed.error(Eigen::VectorXd::Zero(2)).error() == refusal::unsupported_input);
    CHECK(posed.jacobian(Eigen::VectorXd()).error() == refusal::unsupported_input);
}

TEST_CASE("a registration with a box hands that box to its problem and one without hands none", "[optimization][registered]")
{
    const box region{Eigen::VectorXd::Constant(1, -1.0), Eigen::VectorXd::Constant(1, 2.0)};

    const problem bounded   = curve_problem(registered_curve("cubic", &cubic_value, &cubic_slope, region), 1.0e-10);
    const problem unbounded = curve_problem(registered_curve("cubic", &cubic_value, &cubic_slope), 1.0e-10);

    REQUIRE(bounded.bounds.has_value());
    CHECK(bounded.bounds->lower == region.lower);
    CHECK(bounded.bounds->upper == region.upper);
    CHECK(bounded.bounds->period.size() == 0);
    CHECK_FALSE(unbounded.bounds.has_value());
}
