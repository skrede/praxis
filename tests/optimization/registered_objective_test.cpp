#include "answered.h"

#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <limits>

using namespace praxis;
using namespace praxis::optimization;
using praxis::tests::answered;
using praxis::tests::refused;

namespace {

double quartic(double x)
{
    return x * x * x * x - 3.0 * x * x + x;
}

double quartic_slope(double x)
{
    return 4.0 * x * x * x - 6.0 * x + 1.0;
}

double quartic_curvature(double x)
{
    return 12.0 * x * x - 6.0;
}

double bowl(double x, double y)
{
    return x * x + x * y + 2.0 * y * y;
}

Eigen::Vector2d bowl_gradient(double x, double y)
{
    return Eigen::Vector2d(2.0 * x + y, x + 4.0 * y);
}

Eigen::Matrix2d bowl_hessian(double, double)
{
    return (Eigen::Matrix2d() << 2.0, 1.0, 1.0, 4.0).finished();
}

Eigen::VectorXd at(double x)
{
    return Eigen::VectorXd::Constant(1, x);
}

}

TEST_CASE("a curve's objective answers f, its derivative as the gradient and its second derivative as the Hessian", "[optimization][registered]")
{
    const problem posed = curve_objective(registered_curve("quartic", &quartic, &quartic_slope, &quartic_curvature), 1.0e-9);

    for(const double x : {-2.0, -0.5, 0.0, 1.5})
    {
        CHECK(answered(posed.objective(at(x))) == quartic(x));
        const Eigen::VectorXd g = answered(posed.gradient(at(x)));
        const Eigen::MatrixXd h = answered(posed.hessian(at(x)));
        REQUIRE(g.size() == 1);
        REQUIRE((h.rows() == 1 && h.cols() == 1));
        CHECK(g[0] == quartic_slope(x));
        CHECK(h(0, 0) == quartic_curvature(x));
    }
}

TEST_CASE("a curve's objective is stationary exactly where every gradient entry lies inside the tolerance", "[optimization][registered]")
{
    const problem posed = curve_objective(registered_curve("quartic", &quartic, &quartic_slope), 1.0e-9);

    CHECK(posed.stationary(at(0.0)));
    CHECK(posed.stationary(at(-0.9e-9)));
    CHECK_FALSE(posed.stationary(at(1.0e-9)));
    CHECK_FALSE(posed.stationary(at(std::numeric_limits<double>::quiet_NaN())));
    CHECK_FALSE(posed.stationary(Eigen::VectorXd::Zero(2)));
}

TEST_CASE("a curve registered without its second derivative poses an objective carrying no Hessian", "[optimization][registered]")
{
    const problem posed = curve_objective(registered_curve("quartic", &quartic, &quartic_slope), 1.0e-9);

    CHECK(static_cast<bool>(posed.objective));
    CHECK(static_cast<bool>(posed.gradient));
    CHECK(static_cast<bool>(posed.stationary));
    CHECK_FALSE(static_cast<bool>(posed.hessian));
}

TEST_CASE("a curve's objective refuses a value that is not finite as degenerate and an x of the wrong size as unsupported input", "[optimization][registered]")
{
    const registered_curve reciprocal("1/x", [](double x) { return 1.0 / x; }, [](double x) { return -1.0 / (x * x); }, [](double x) { return 2.0 / (x * x * x); });
    const problem posed = curve_objective(reciprocal, 1.0e-9);

    CHECK(refused(posed.objective(at(0.0))) == refusal::degenerate);
    CHECK(refused(posed.gradient(at(0.0))) == refusal::degenerate);
    CHECK(refused(posed.hessian(at(0.0))) == refusal::degenerate);
    CHECK(refused(posed.objective(Eigen::VectorXd::Ones(2))) == refusal::unsupported_input);
    CHECK(refused(posed.gradient(Eigen::VectorXd::Ones(2))) == refusal::unsupported_input);
    CHECK(refused(posed.hessian(Eigen::VectorXd())) == refusal::unsupported_input);
}

TEST_CASE("a surface's objective answers f, its gradient and its two by two Hessian", "[optimization][registered]")
{
    const problem posed = surface_objective(registered_surface("bowl", &bowl, &bowl_gradient, &bowl_hessian), 1.0e-9);
    const Eigen::Vector2d x(0.5, -1.25);

    CHECK(answered(posed.objective(x)) == bowl(0.5, -1.25));
    CHECK(answered(posed.gradient(x)) == Eigen::VectorXd(bowl_gradient(0.5, -1.25)));
    CHECK(answered(posed.hessian(x)) == Eigen::MatrixXd(bowl_hessian(0.5, -1.25)));
    CHECK(posed.stationary(Eigen::Vector2d(0.5e-9, -0.5e-9)));
    CHECK_FALSE(posed.stationary(Eigen::Vector2d(0.0, 2.0e-9)));
}

TEST_CASE("a surface registered without its Hessian poses an objective carrying none, and its box", "[optimization][registered]")
{
    const box region{Eigen::Vector2d(-1.0, -2.0), Eigen::Vector2d(1.0, 2.0)};
    const problem posed = surface_objective(registered_surface("bowl", &bowl, &bowl_gradient, region), 1.0e-9);

    CHECK_FALSE(static_cast<bool>(posed.hessian));
    REQUIRE(posed.bounds.has_value());
    CHECK(posed.bounds->lower == region.lower);
    CHECK(posed.bounds->upper == region.upper);
}

TEST_CASE("a surface's objective refuses a value that is not finite as degenerate and an x of the wrong size as unsupported input", "[optimization][registered]")
{
    const double inf = std::numeric_limits<double>::infinity();
    const registered_surface steep(
            "steep", [inf](double, double) { return inf; }, [inf](double, double) { return Eigen::Vector2d(inf, 0.0); },
            [inf](double, double) { return Eigen::Matrix2d(Eigen::Matrix2d::Constant(inf)); });
    const problem posed = surface_objective(steep, 1.0e-9);

    CHECK(refused(posed.objective(Eigen::Vector2d::Zero())) == refusal::degenerate);
    CHECK(refused(posed.gradient(Eigen::Vector2d::Zero())) == refusal::degenerate);
    CHECK(refused(posed.hessian(Eigen::Vector2d::Zero())) == refusal::degenerate);
    CHECK(refused(posed.objective(at(0.0))) == refusal::unsupported_input);
    CHECK(refused(posed.hessian(Eigen::VectorXd::Zero(3))) == refusal::unsupported_input);
}
