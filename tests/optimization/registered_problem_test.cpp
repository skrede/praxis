#include "answered.h"

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

    CHECK(tests::refused(unvalued.error(at(1.0))) == refusal::degenerate);
    CHECK(unvalued.jacobian(at(1.0)));

    CHECK(tests::refused(unsloped.jacobian(at(1.0))) == refusal::degenerate);
    CHECK(unsloped.error(at(1.0)));
}

TEST_CASE("a registered curve's problem refuses a missing function and an x of other than one coordinate as unsupported input", "[optimization][registered]")
{
    const problem unbound = curve_problem(registered_curve("unbound", {}, {}), 1.0e-10);
    const problem posed   = curve_problem(registered_curve("cubic", &cubic_value, &cubic_slope), 1.0e-10);

    CHECK(tests::refused(unbound.error(at(1.0))) == refusal::unsupported_input);
    CHECK(tests::refused(unbound.jacobian(at(1.0))) == refusal::unsupported_input);
    CHECK(tests::refused(posed.error(Eigen::VectorXd::Zero(2))) == refusal::unsupported_input);
    CHECK(tests::refused(posed.jacobian(Eigen::VectorXd())) == refusal::unsupported_input);
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

namespace {

double circle_value(double x, double y)
{
    return x * x + y * y - 1.0;
}

Eigen::Vector2d circle_gradient(double x, double y)
{
    return Eigen::Vector2d(2.0 * x, 2.0 * y);
}

Eigen::VectorXd at(double x, double y)
{
    return Eigen::Vector2d(x, y);
}

}

TEST_CASE("a registered surface's problem answers the residual -f and its gradient as one row", "[optimization][registered]")
{
    const problem posed = surface_problem(registered_surface("circle", &circle_value, &circle_gradient), 1.0e-3);

    for(const Eigen::Vector2d &x : {Eigen::Vector2d(-2.0, 0.5), Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.5, -3.0)})
    {
        const expected<Eigen::VectorXd, refusal> e = posed.error(x);
        const expected<Eigen::MatrixXd, refusal> j = posed.jacobian(x);

        REQUIRE(e);
        REQUIRE(j);
        REQUIRE(e->size() == 1);
        REQUIRE(j->rows() == 1);
        REQUIRE(j->cols() == 2);
        CHECK((*e)[0] == -circle_value(x[0], x[1]));
        CHECK((*j)(0, 0) == 2.0 * x[0]);
        CHECK((*j)(0, 1) == 2.0 * x[1]);
    }

    CHECK(posed.converged(Eigen::VectorXd::Constant(1, 1.0e-4)));
    CHECK_FALSE(posed.converged(Eigen::VectorXd::Constant(1, 1.0e-2)));
    CHECK_FALSE(posed.bounds.has_value());
}

TEST_CASE("a registered surface's problem refuses a value or a gradient that is not finite as degenerate", "[optimization][registered]")
{
    const std::function<double(double, double)> undefined          = [](double, double) { return std::numeric_limits<double>::quiet_NaN(); };
    const std::function<Eigen::Vector2d(double, double)> unbounded = [](double, double) { return Eigen::Vector2d(1.0, std::numeric_limits<double>::infinity()); };

    const problem unvalued = surface_problem(registered_surface("undefined", undefined, &circle_gradient), 1.0e-10);
    const problem unsloped = surface_problem(registered_surface("unbounded", &circle_value, unbounded), 1.0e-10);

    CHECK(tests::refused(unvalued.error(at(1.0, 2.0))) == refusal::degenerate);
    CHECK(unvalued.jacobian(at(1.0, 2.0)));

    CHECK(tests::refused(unsloped.jacobian(at(1.0, 2.0))) == refusal::degenerate);
    CHECK(unsloped.error(at(1.0, 2.0)));
}

TEST_CASE("a registered surface's problem refuses a missing function and an x of other than two coordinates as unsupported input", "[optimization][registered]")
{
    const box region{Eigen::Vector2d(-1.0, -1.0), Eigen::Vector2d(1.0, 2.0)};

    const problem unbound = surface_problem(registered_surface("unbound", {}, {}), 1.0e-10);
    const problem boxed   = surface_problem(registered_surface("circle", &circle_value, &circle_gradient, region), 1.0e-10);

    CHECK(tests::refused(unbound.error(at(1.0, 2.0))) == refusal::unsupported_input);
    CHECK(tests::refused(unbound.jacobian(at(1.0, 2.0))) == refusal::unsupported_input);
    CHECK(tests::refused(boxed.error(Eigen::VectorXd::Zero(1))) == refusal::unsupported_input);
    CHECK(tests::refused(boxed.jacobian(Eigen::VectorXd::Zero(3))) == refusal::unsupported_input);
    REQUIRE(boxed.bounds.has_value());
    CHECK(boxed.bounds->upper == region.upper);
}
