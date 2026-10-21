#include "drawn_surface.h"
#include "composed_curve.h"
#include "drawn_iterates.h"
#include "scenario_defaults.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/scenario_state.h"
#include "praxis/optimization/scenario_snapshot.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <functional>

using namespace praxis;
using namespace praxis::fixture;

namespace {

const field_placement identity{Eigen::Vector2d(0.0, 0.0), 1.0, 1.0};
constexpr double apart = 0.1;

// Each point (x, y, z) is an iterate at (x, y) whose error is -z, so it is placed at the point itself.
scenario_snapshot run_through(const std::vector<Eigen::Vector3d> &points, std::uint32_t step)
{
    scenario_snapshot shown{};
    shown.step   = step;
    shown.ranges = {interval{-2.0, 2.0}, interval{-2.0, 2.0}};
    for(const Eigen::Vector3d &point : points)
    {
        shown.run.iterates.emplace_back(Eigen::Vector2d(point.x(), point.y()));
        shown.run.errors.emplace_back(Eigen::VectorXd::Constant(1, -point.z()));
    }

    return shown;
}

std::vector<std::size_t> order(const scenario_snapshot &shown)
{
    std::vector<std::size_t> drawn;
    for(const drawn_iterate &mark : drawn_iterates(shown, identity, apart))
        drawn.push_back(mark.iterate);

    return drawn;
}

using indices = std::vector<std::size_t>;

double paraboloid(double x, double y)
{
    return x * x + y * y - 1.0;
}

std::shared_ptr<const scenario_snapshot> drawn_from(scheduler::scheduler &loop, const staged &built, opened &taken, const Eigen::Vector2d &seed, std::uint32_t limit)
{
    const scenario_settings settings{"paraboloid", {interval{-2.0, 2.0}, interval{-2.0, 2.0}}, seed, limit, 0u};
    taken = open_surface_with(built.site, {registered_surface("paraboloid", &paraboloid, [](double x, double y) { return Eigen::Vector2d(2.0 * x, 2.0 * y); })}, baseline(), settings);
    const std::shared_ptr<const scenario_snapshot> ran = settled(loop, taken);
    taken.composed->stencil->render();
    return ran;
}

void commanded(scheduler::scheduler &loop, const opened &taken, const std::function<void(scenario_state &)> &change)
{
    command(taken.inputs->scenario, change);
    REQUIRE(loop.drain().has_value());
    taken.composed->stencil->render();
}

Eigen::Vector3d standing_at(threepp::Scene &scene, const scenario_snapshot &ran, std::size_t k)
{
    const Eigen::VectorXd &x = ran.run.iterates[k];

    return placement_of(field_of(scene), ran).at(x[0], x[1], paraboloid(x[0], x[1]));
}

void check_selected_last(const threepp::InstancedMesh &marks)
{
    REQUIRE(marks.count() >= 1u);
    for(std::size_t j = 0; j + 1u < marks.count(); ++j)
        CHECK(tone_at(marks, j) == threepp::Color(iterate_tone));
    CHECK(tone_at(marks, marks.count() - 1u) == threepp::Color(selected_iterate_tone));
}

void check_drawn(threepp::Scene &scene, const scenario_snapshot &ran, const indices &expected)
{
    const threepp::InstancedMesh *marks = iterate_marks(scene);
    REQUIRE(marks != nullptr);
    CHECK(static_cast<std::size_t>(marks->instanceMatrix()->count()) == expected.size());
    const std::vector<Eigen::Vector3d> markers = shown_markers(scene);
    REQUIRE(markers.size() == expected.size());
    for(std::size_t j = 0; j < markers.size(); ++j)
        CHECK((markers[j] - standing_at(scene, ran, expected[j])).norm() < 1.0e-5);
    check_selected_last(*marks);
}

}

TEST_CASE("coincident iterates draw the first, the last and the selected one, the selected one last", "[optimization][surface]")
{
    const std::vector<Eigen::Vector3d> together(5u, Eigen::Vector3d(0.3, 0.3, 0.0));
    CHECK(order(run_through(together, 0u)) == indices{4u, 0u});
    CHECK(order(run_through(together, 2u)) == indices{0u, 4u, 2u});
    const std::vector<Eigen::Vector3d> stalled(100000u, Eigen::Vector3d(0.0, 0.0, -1.0));
    CHECK(order(run_through(stalled, 0u)) == indices{99999u, 0u});
    CHECK(order(run_through({}, 0u)).empty());
}

TEST_CASE("iterates standing apart are all drawn and one within the spacing of a drawn one is not, across a cell's boundary too", "[optimization][surface]")
{
    std::vector<Eigen::Vector3d> along_x, along_z;
    for(std::size_t k = 0; k < 5u; ++k)
    {
        along_x.emplace_back(1.01 * apart * static_cast<double>(k), 0.0, 0.0);
        along_z.emplace_back(0.0, 0.0, 1.01 * apart * static_cast<double>(k));
    }
    CHECK(order(run_through(along_x, 2u)) == indices{0u, 1u, 3u, 4u, 2u});
    CHECK(order(run_through(along_z, 2u)) == indices{0u, 1u, 3u, 4u, 2u});
    CHECK(order(run_through({Eigen::Vector3d::Zero(), Eigen::Vector3d(0.01, 0.0, 0.0)}, 0u)) == indices{1u, 0u});

    const std::vector<std::array<Eigen::Vector3d, 2>> close{{Eigen::Vector3d::Zero(), Eigen::Vector3d(0.99 * apart, 0.0, 0.0)},
                                                            {Eigen::Vector3d(0.095, 0.0, 0.0), Eigen::Vector3d(0.105, 0.0, 0.0)},
                                                            {Eigen::Vector3d(-0.005, 0.0, 0.0), Eigen::Vector3d(0.005, 0.0, 0.0)},
                                                            {Eigen::Vector3d(0.0, 0.0, 0.095), Eigen::Vector3d(0.0, 0.0, 0.105)},
                                                            {Eigen::Vector3d(0.0, 0.0, 1.0e300), Eigen::Vector3d(0.0, 0.0, 1.0e300)}};
    for(const std::array<Eigen::Vector3d, 2> &pair : close)
        CHECK(order(run_through({pair[0], pair[1], Eigen::Vector3d(1.0, 1.0, 0.0)}, 0u)) == indices{2u, 0u});
}

TEST_CASE("an iterate outside the ranges or whose f was refused is not drawn, even when last or selected", "[optimization][surface]")
{
    scenario_snapshot outside = run_through({Eigen::Vector3d::Zero(), Eigen::Vector3d(0.5, 0.5, 0.0), Eigen::Vector3d(3.0, 0.0, 0.0)}, 1u);
    outside.run.iterates[1]   = Eigen::Vector2d(0.5, 2.5);
    CHECK(order(outside) == indices{0u});

    scenario_snapshot refused = run_through({Eigen::Vector3d::Zero(), Eigen::Vector3d(0.5, 0.5, 0.0), Eigen::Vector3d(1.0, 1.0, 0.0)}, 1u);
    refused.run.errors[1]     = praxis::unexpected(refusal::no_solution);
    refused.run.errors[2]     = praxis::unexpected(refusal::no_solution);
    CHECK(order(refused) == indices{0u});
}

TEST_CASE("a stalled surface run draws two markers at its seed, and a step selected inside it a third, written last", "[optimization][surface]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    opened taken;
    const std::shared_ptr<const scenario_snapshot> ran = drawn_from(loop, built, taken, Eigen::Vector2d(0.0, 0.0), 100000u);
    REQUIRE(ran->run.iterates.size() == 100001u);
    check_drawn(*built.scene, *ran, indices{100000u, 0u});

    commanded(loop, taken, [](scenario_state &state) { state.select_step(50000u); });
    check_drawn(*built.scene, *ran, indices{0u, 100000u, 50000u});

    taken.composed->tear_down();
}

TEST_CASE("a surface's markers are one mesh, the selected step's last, rewritten only on a new publication", "[optimization][surface]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    opened taken;
    const std::shared_ptr<const scenario_snapshot> ran = drawn_from(loop, built, taken, Eigen::Vector2d(1.5, 0.5), 70u);
    REQUIRE(ran->run.iterates.size() > 2u);
    const threepp::InstancedMesh *marks = iterate_marks(*built.scene);
    REQUIRE(marks != nullptr);
    check_selected_last(*marks);
    const unsigned int written = marks->instanceMatrix()->version;
    taken.composed->stencil->render();
    CHECK(marks->instanceMatrix()->version == written);

    commanded(loop, taken, [](scenario_state &state) { state.select_step(1u); });
    const threepp::InstancedMesh *selected = iterate_marks(*built.scene);
    REQUIRE(selected != nullptr);
    CHECK((selected != marks || selected->instanceMatrix()->version != written));
    CHECK((shown_markers(*built.scene).back() - standing_at(*built.scene, *ran, 1u)).norm() < 1.0e-5);
    check_selected_last(*selected);

    taken.composed->tear_down();
}

TEST_CASE("a shorter run leaves one mesh holding its own two markers", "[optimization][surface]")
{
    scheduler::scheduler loop(scheduler::inline_workers);
    staged built(loop);
    opened taken;
    drawn_from(loop, built, taken, Eigen::Vector2d(1.5, 0.5), 70u);
    commanded(loop, taken, [](scenario_state &state) { state.run(Eigen::Vector2d(1.5, 0.5), 1u); });
    CHECK(built.scene->getObjectByName("iterates")->children.size() == 1u);
    CHECK(shown_markers(*built.scene).size() == 2u);
    taken.composed->tear_down();
}
