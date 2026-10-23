#include "answered.h"

#include "typed/parse.h"
#include "typed/evaluate.h"
#include "typed/typed_problem.h"

#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/baseline/solver.h"
#include "praxis/optimization/baseline/minimizer.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <vector>
#include <cstddef>
#include <string_view>

using namespace praxis;
using namespace praxis::optimization;
using praxis::tests::answered;
using praxis::tests::refused;

namespace {

std::shared_ptr<const typed::expression> tree_of(std::string_view text, std::size_t dimension)
{
    expected<typed::expression, typed::parse_failure> read = typed::parsed(text, dimension);
    REQUIRE(read.has_value());
    return std::make_shared<const typed::expression>(std::move(*read));
}

Eigen::VectorXd at(double x)
{
    return Eigen::VectorXd::Constant(1, x);
}

}

TEST_CASE("the typed objective of a quartic reaches the registered quartic's minimizer through the Newton reference", "[optimization][typed]")
{
    const problem typed = typed::typed_objective(tree_of("x^4 - 3x^2 + x", 1), typed::parameter_values{}, 1.0e-12);
    const registered_curve quartic(
            "quartic", [](double x) { return x * x * x * x - 3.0 * x * x + x; }, [](double x) { return 4.0 * x * x * x - 6.0 * x + 1.0; }, [](double x) { return 12.0 * x * x - 6.0; });
    const problem written = curve_objective(quartic, 1.0e-12);

    std::vector<Eigen::VectorXd> typed_iterates;
    std::vector<Eigen::VectorXd> written_iterates;
    const Eigen::VectorXd reached  = answered(newton(typed, at(2.0), 50, typed_iterates));
    const Eigen::VectorXd expected = answered(newton(written, at(2.0), 50, written_iterates));

    CHECK(std::abs(reached[0] - expected[0]) < 1.0e-12);
    REQUIRE_FALSE(typed_iterates.empty());
    REQUIRE_FALSE(written_iterates.empty());
    CHECK(typed_iterates.front()[0] == 2.0);
    CHECK(written_iterates.front()[0] == 2.0);
}

TEST_CASE("the typed residual of a cubic reaches the registered cubic's root through the Newton-Raphson reference", "[optimization][typed]")
{
    const problem typed = typed::typed_residual(tree_of("x^3 - 2x - 5", 1), typed::parameter_values{}, 1.0e-12);
    const registered_curve cubic("cubic", [](double x) { return x * x * x - 2.0 * x - 5.0; }, [](double x) { return 3.0 * x * x - 2.0; });
    const problem written = curve_problem(cubic, 1.0e-12);

    std::vector<Eigen::VectorXd> typed_iterates;
    std::vector<Eigen::VectorXd> written_iterates;
    const Eigen::VectorXd reached  = answered(solve(typed, at(3.0), 50, typed_iterates));
    const Eigen::VectorXd expected = answered(solve(written, at(3.0), 50, written_iterates));

    CHECK(std::abs(reached[0] - expected[0]) < 1.0e-12);
    REQUIRE_FALSE(typed_iterates.empty());
    REQUIRE_FALSE(written_iterates.empty());
    CHECK(typed_iterates.front()[0] == 3.0);
    CHECK(written_iterates.front()[0] == 3.0);
}

TEST_CASE("a typed objective refuses as degenerate wherever the value, the gradient or the Hessian it answers is not finite", "[optimization][typed]")
{
    const problem root   = typed::typed_objective(tree_of("x^0.5", 1), typed::parameter_values{}, 1.0e-9);
    const problem log    = typed::typed_objective(tree_of("log(x)", 1), typed::parameter_values{}, 1.0e-9);
    const problem self   = typed::typed_objective(tree_of("x^x", 1), typed::parameter_values{}, 1.0e-9);
    const problem sloped = typed::typed_objective(tree_of("sqrt(x) + y", 2), typed::parameter_values{}, 1.0e-9);

    CHECK(answered(root.objective(at(0.0))) == 0.0);
    CHECK(refused(root.gradient(at(0.0))) == refusal::degenerate);
    CHECK(refused(root.hessian(at(0.0))) == refusal::degenerate);
    CHECK(refused(log.objective(at(0.0))) == refusal::degenerate);
    CHECK(refused(log.gradient(at(0.0))) == refusal::degenerate);
    CHECK(refused(self.gradient(at(-1.0))) == refusal::degenerate);
    CHECK(refused(self.hessian(at(-1.0))) == refusal::degenerate);
    CHECK(answered(sloped.objective(Eigen::Vector2d(0.0, 1.0))) == 1.0);
    CHECK(refused(sloped.gradient(Eigen::Vector2d(0.0, 1.0))) == refusal::degenerate);
}

TEST_CASE("a typed residual refuses a value that is not finite as degenerate", "[optimization][typed]")
{
    const problem reciprocal = typed::typed_residual(tree_of("1/x", 1), typed::parameter_values{}, 1.0e-9);

    CHECK(refused(reciprocal.error(at(0.0))) == refusal::degenerate);
    CHECK(refused(reciprocal.jacobian(at(0.0))) == refusal::degenerate);
    CHECK(answered(reciprocal.error(at(2.0)))[0] == -0.5);
}

TEST_CASE("a typed residual and objective refuse an x that is not one coordinate per variable as unsupported input", "[optimization][typed]")
{
    const std::shared_ptr<const typed::expression> tree = tree_of("x^2 + y", 2);
    const problem residual                              = typed::typed_residual(tree, typed::parameter_values{}, 1.0e-9);
    const problem objective                             = typed::typed_objective(tree, typed::parameter_values{}, 1.0e-9);
    const Eigen::VectorXd three                         = Eigen::VectorXd::Ones(3);

    CHECK(refused(residual.error(three)) == refusal::unsupported_input);
    CHECK(refused(residual.jacobian(three)) == refusal::unsupported_input);
    CHECK(refused(objective.objective(three)) == refusal::unsupported_input);
    CHECK(refused(objective.gradient(three)) == refusal::unsupported_input);
    CHECK(refused(objective.hessian(three)) == refusal::unsupported_input);
    CHECK(answered(residual.jacobian(Eigen::Vector2d(1.5, 0.0))) == Eigen::RowVector2d(3.0, 1.0));
}

TEST_CASE("a typed problem reads each parameter from the values handed in", "[optimization][typed]")
{
    const std::shared_ptr<const typed::expression> tree = tree_of("b x^2", 1);
    typed::parameter_values values{};
    values['b' - 'a']      = 2.5;
    const problem scaled   = typed::typed_objective(tree, values, 1.0e-9);
    const problem unscaled = typed::typed_objective(tree, typed::parameter_values{}, 1.0e-9);

    CHECK(std::abs(answered(scaled.objective(at(0.9))) - 2.025) < 1.0e-15);
    CHECK(std::abs(answered(scaled.gradient(at(0.9)))[0] - 4.5) < 1.0e-15);
    CHECK(answered(scaled.hessian(at(0.9)))(0, 0) == 5.0);
    CHECK(answered(unscaled.objective(at(0.9))) == 0.0);
    CHECK(answered(unscaled.gradient(at(0.9)))[0] == 0.0);
    CHECK(answered(unscaled.hessian(at(0.9)))(0, 0) == 0.0);
    CHECK(unscaled.stationary(Eigen::VectorXd::Zero(1)));
}

TEST_CASE("a typed function smooth where a rule inside it is singular is solved and minimized from a seed beside it", "[optimization][typed]")
{
    const typed::parameter_values zero{};
    std::vector<Eigen::VectorXd> iterates;
    CHECK(std::abs(answered(solve(typed::typed_residual(tree_of("sqrt(a)*x^2 + x", 1), zero, 1.0e-12), at(0.5), 50, iterates))[0]) < 1.0e-12);
    CHECK(std::abs(answered(newton(typed::typed_objective(tree_of("a^1.5*x^2 + x^2", 1), zero, 1.0e-12), at(0.5), 50, iterates))[0]) < 1.0e-12);
}
