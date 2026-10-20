#include "drawn_surface.h"
#include "surface_step.h"
#include "composed_curve.h"

#include "praxis/optimization/problem.h"
#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/optimization/baseline/solver.h"
#include "praxis/optimization/projected/solver.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <cstddef>
#include <optional>

using namespace praxis;
using namespace praxis::fixture;

namespace {

const std::vector<interval> square{interval{-2.0, 2.0}, interval{-2.0, 2.0}};

double circle_value(double x, double y)
{
    return x * x + y * y - 1.0;
}

Eigen::Vector2d circle_gradient(double x, double y)
{
    return Eigen::Vector2d(2.0 * x, 2.0 * y);
}

registered_surface circle()
{
    return registered_surface("circle", &circle_value, &circle_gradient);
}

scenario_settings opening_at(const Eigen::Vector2d &seed)
{
    return scenario_settings{"circle", square, seed, 70u, 0u};
}

double plane_at(const Eigen::Vector2d &x, double f, const Eigen::Vector2d &gradient, const Eigen::Vector2d &q)
{
    return f + gradient.dot(q - x);
}

// The construction from x against the iterate the solve recorded after it.
void check_step(const Eigen::Vector2d &x, const Eigen::Vector2d &next)
{
    const double f                               = circle_value(x.x(), x.y());
    const Eigen::Vector2d gradient               = circle_gradient(x.x(), x.y());
    const double rounding                        = 1.0e-12 * std::max(1.0, std::abs(f));
    const std::optional<step_construction> built = constructed_step(x, f, gradient, square);
    REQUIRE(built.has_value());
    CHECK((built->nearest - next).norm() <= 1.0e-12);

    REQUIRE(built->zero_line.has_value());
    const std::array<Eigen::Vector2d, 2> &line = *built->zero_line;
    const Eigen::Vector2d along                = line[1] - line[0];
    CHECK(std::abs(along.dot(gradient)) <= 1.0e-12 * std::max(1.0, along.norm()) * gradient.norm());
    for(const Eigen::Vector2d &end : line)
        CHECK(std::abs(plane_at(x, f, gradient, end)) <= rounding);
    CHECK(std::abs((built->nearest - line[0]).norm() + (line[1] - built->nearest).norm() - along.norm()) <= 1.0e-12);

    REQUIRE(built->patch.size() >= 3u);
    for(const Eigen::Vector3d &corner : built->patch)
        CHECK(std::abs(corner.z() - plane_at(x, f, gradient, corner.head<2>())) <= rounding);
}

void check_steps_from(const Eigen::Vector2d &seed)
{
    std::vector<Eigen::VectorXd> iterates;
    REQUIRE(optimization::solve(surface_problem(circle(), 1.0e-12), seed, 70u, iterates).has_value());
    REQUIRE(iterates.size() >= 3u);
    for(std::size_t k = 0; k + 1 < iterates.size(); ++k)
        check_step(iterates[k], iterates[k + 1]);
}

struct composed_surface
{
    scheduler::scheduler loop;
    staged built;
    opened taken;
    std::shared_ptr<const scenario_snapshot> ran;

    composed_surface(registered_surface surface, const capabilities &solving, const Eigen::Vector2d &seed)
            : loop(scheduler::inline_workers)
            , built(loop)
            , taken(open_surface_with(built.site, {std::move(surface)}, solving, opening_at(seed)))
            , ran(settled(loop, taken))
    {
        taken.composed->stencil->render();
    }

    ~composed_surface()
    {
        taken.composed->tear_down();
    }

    void select(std::uint32_t step)
    {
        command(taken.inputs->scenario, [step](scenario_state &state) { state.select_step(step); });
        REQUIRE(loop.drain().has_value());
        ran = taken.inputs->seen.read();
        taken.composed->stencil->render();
    }
};

}

TEST_CASE("at every step of the reference solve the nearest point of the tangent plane's zero line is the next iterate", "[optimization][surface][step]")
{
    check_steps_from(Eigen::Vector2d(1.5, 0.5));
    check_steps_from(Eigen::Vector2d(-1.75, 0.25));
    check_steps_from(Eigen::Vector2d(0.3, -1.9));
}

TEST_CASE("the stencil draws the selected step's tangent plane, its zero line and the nearest point on the zero plane", "[optimization][surface][step]")
{
    composed_surface staged_surface(circle(), baseline(), Eigen::Vector2d(1.5, 0.5));
    staged_surface.select(1u);
    const scenario_snapshot &ran = *staged_surface.ran;
    REQUIRE(ran.step == 1u);

    threepp::Scene &scene       = *staged_surface.built.scene;
    const placement standing    = placement_of(field_of(scene), *ran.grid);
    const Eigen::VectorXd &next = ran.run.iterates[2];
    CHECK((drawn_position(scene, "nearest point") - standing.at(next[0], next[1], 0.0)).norm() <= 1.0e-5);
    CHECK(drawn_visible(scene, "tangent plane"));

    const std::vector<Eigen::Vector3d> line = drawn_points(scene, "zero line");
    REQUIRE(line.size() == 2u);
    for(const Eigen::Vector3d &end : line)
        CHECK(std::abs(end.z()) <= 1.0e-6);
    const Eigen::Vector3d nearest = drawn_position(scene, "nearest point");
    CHECK(std::abs((nearest - line[0]).norm() + (line[1] - nearest).norm() - (line[1] - line[0]).norm()) <= 1.0e-5);
}

TEST_CASE("under the projected solver in a box the recorded next iterate is drawn apart from the nearest point", "[optimization][surface][step]")
{
    const Eigen::Vector2d unbounded = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
    const box region{Eigen::Vector2d(-unbounded.x(), -2.0), Eigen::Vector2d(1.0, unbounded.y())};
    composed_surface staged_surface(registered_surface("circle", &circle_value, &circle_gradient, region), capabilities{solver_ops{&projected::solve}}, Eigen::Vector2d(1.5, 0.5));
    const scenario_snapshot &ran = *staged_surface.ran;
    REQUIRE(ran.run.iterates.size() >= 2u);

    const Eigen::Vector2d seed                   = ran.run.iterates[0];
    const std::optional<step_construction> built = constructed_step(seed, circle_value(seed.x(), seed.y()), circle_gradient(seed.x(), seed.y()), square);
    REQUIRE(built.has_value());
    CHECK((built->nearest - ran.run.iterates[1]).norm() > 1.0e-3);

    threepp::Scene &scene       = *staged_surface.built.scene;
    const placement standing    = placement_of(field_of(scene), *ran.grid);
    const Eigen::VectorXd &next = ran.run.iterates[1];
    CHECK(next[0] == region.upper.x());
    const std::vector<Eigen::Vector3d> recorded = drawn_points(scene, "x_(k+1)");
    REQUIRE(recorded.size() == 2u);
    CHECK((recorded[0] - standing.at(next[0], next[1], 0.0)).norm() <= 1.0e-5);
    CHECK((recorded[1] - standing.at(next[0], next[1], circle_value(next[0], next[1]))).norm() <= 1.0e-5);
    CHECK((drawn_position(scene, "nearest point") - standing.at(built->nearest.x(), built->nearest.y(), 0.0)).norm() <= 1.0e-5);
    CHECK((drawn_position(scene, "nearest point") - recorded[0]).norm() > 1.0e-4);
}

TEST_CASE("a zero gradient, or a point already on the zero line, has no zero line and no nearest point", "[optimization][surface][step]")
{
    CHECK_FALSE(constructed_step(Eigen::Vector2d(0.0, 0.0), -1.0, Eigen::Vector2d(0.0, 0.0), square).has_value());
    CHECK_FALSE(constructed_step(Eigen::Vector2d(1.0, 0.0), 0.0, Eigen::Vector2d(2.0, 0.0), square).has_value());

    composed_surface staged_surface(circle(), baseline(), Eigen::Vector2d(0.0, 0.0));
    REQUIRE(staged_surface.ran->run.refused.has_value());
    threepp::Scene &scene = *staged_surface.built.scene;
    CHECK_FALSE(drawn_visible(scene, "tangent plane"));
    CHECK_FALSE(drawn_visible(scene, "zero line"));
    CHECK_FALSE(drawn_visible(scene, "nearest point"));
}
