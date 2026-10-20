#include "demo_offered.h"
#include "demo_configuration.h"

#include "praxis/presets/optimization.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"

#include "praxis/config/store.h"

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>

namespace praxis::demo {

namespace {

double cubic(double x)
{
    return x * x * x - 2.0 * x - 5.0;
}

double cubic_slope(double x)
{
    return 3.0 * x * x - 2.0;
}

double cosine_less_x(double x)
{
    return std::cos(x) - x;
}

double cosine_less_x_slope(double x)
{
    return -std::sin(x) - 1.0;
}

double arctangent(double x)
{
    return std::atan(x);
}

double arctangent_slope(double x)
{
    return 1.0 / (1.0 + x * x);
}

double paraboloid(double x, double y)
{
    return x * x + y * y - 1.0;
}

Eigen::Vector2d paraboloid_gradient(double x, double y)
{
    return Eigen::Vector2d(2.0 * x, 2.0 * y);
}

double hyperbolic_paraboloid(double x, double y)
{
    return x * y - 1.0;
}

Eigen::Vector2d hyperbolic_paraboloid_gradient(double x, double y)
{
    return Eigen::Vector2d(y, x);
}

std::vector<optimization::registered_curve> example_curves()
{
    return {optimization::registered_curve("x^3 - 2x - 5", &cubic, &cubic_slope), optimization::registered_curve("cos(x) - x", &cosine_less_x, &cosine_less_x_slope),
            optimization::registered_curve("atan(x)", &arctangent, &arctangent_slope)};
}

std::vector<optimization::registered_surface> example_surfaces()
{
    return {optimization::registered_surface("x^2 + y^2 - 1", &paraboloid, &paraboloid_gradient),
            optimization::registered_surface("x y - 1", &hyperbolic_paraboloid, &hyperbolic_paraboloid_gradient)};
}

}

std::vector<std::string> register_offered(const std::shared_ptr<scene::preset_registry> &registry, const documents &mine, const config::document &values, const std::filesystem::path &,
                                          const std::shared_ptr<write_back> &writing)
{
    const std::vector<config::location> reading = preset_locations(values, mine);
    const bool checks_derivative                = true;

    return presets::register_optimization(
            registry, reading, [mine](const std::filesystem::path &named) { return mine.reading(named); }, [writing](const config::binding &at, const config::document &carried)
            { writing->composing(at, carried); }, optimization::baseline(), example_curves(), example_surfaces(), checks_derivative);
}

}
