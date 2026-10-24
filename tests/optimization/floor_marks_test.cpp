#include "drawn_surface.h"
#include "minimized_surface.h"
#include "scenario_defaults.h"

#include "praxis/optimization/slots.h"
#include "praxis/optimization/problem.h"
#include "praxis/optimization/scenario_snapshot.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/math/Color.hpp>

#include <threepp/objects/InstancedMesh.hpp>

#include <Eigen/Core>

#include <vector>
#include <cstddef>
#include <cstdint>

using namespace praxis;
using namespace praxis::fixture;

namespace {

scenario_settings descended()
{
    return scenario_settings{"(x^2 + 10y^2)/2",
                             {interval{-2.0, 2.0}, interval{-2.0, 2.0}},
                             Eigen::Vector2d(-2.0, -2.0),
                             500u,
                             0u,
                             objective_settings{problem_form::minimum, minimizer_slot::gradient_descent, 0.1},
                             {}};
}

const threepp::InstancedMesh &floor_marks(threepp::Scene &scene)
{
    threepp::Object3D *projected = scene.getObjectByName("iterates on the floor");
    REQUIRE(projected != nullptr);
    REQUIRE(projected->children.size() == 1u);
    const auto *marks = projected->children.front()->as<threepp::InstancedMesh>();
    REQUIRE(marks != nullptr);

    return *marks;
}

// The floor instance whose center stands under x_k.
std::size_t under(threepp::Scene &scene, const scenario_snapshot &shown, std::size_t k)
{
    const Eigen::VectorXd &x                   = shown.run.iterates[k];
    const Eigen::Vector3d over                 = placement_of(field_of(scene), shown).at(x[0], x[1], 0.0);
    const std::vector<Eigen::Vector3d> centers = projected_centers(scene);
    for(std::size_t j = 0; j < centers.size(); ++j)
        if((centers[j].head<2>() - over.head<2>()).norm() <= 1.0e-5)
            return j;
    FAIL("no floor marker stands under x_" << k);

    return centers.size();
}

void check_floor_tones(threepp::Scene &scene, std::size_t highlighted)
{
    const threepp::InstancedMesh &marks = floor_marks(scene);
    REQUIRE(highlighted < marks.count());
    for(std::size_t j = 0; j < marks.count(); ++j)
        CHECK(tone_at(marks, j) == threepp::Color(j == highlighted ? floor_selected_iterate_tone : floor_iterate_tone));
}

void check_field_tones(threepp::Scene &scene)
{
    const threepp::InstancedMesh *marks = iterate_marks(scene);
    REQUIRE(marks != nullptr);
    for(std::size_t j = 0; j + 1u < marks->count(); ++j)
        CHECK(tone_at(*marks, j) == threepp::Color(iterate_tone));
    CHECK(tone_at(*marks, marks->count() - 1u) == threepp::Color(selected_iterate_tone));
}

}

TEST_CASE("the floor marker of the selected step's iterate wears the highlight and the others the floor tone", "[optimization][surface][objective]")
{
    minimized_surface drawn(elliptic(), descended());
    REQUIRE(drawn.shown()->step == 0u);
    REQUIRE(drawn.shown()->run.iterates.size() > 7u);

    const std::size_t last = floor_marks(drawn.scene()).count() - 1u;
    CHECK(under(drawn.scene(), *drawn.shown(), 0u) == last);
    check_floor_tones(drawn.scene(), last);
    check_field_tones(drawn.scene());
}

TEST_CASE("the floor's highlight follows the selected step", "[optimization][surface][objective]")
{
    minimized_surface drawn(elliptic(), descended());
    drawn.select(5u);
    REQUIRE(drawn.shown()->step == 5u);

    const std::size_t selected = under(drawn.scene(), *drawn.shown(), 5u);
    const std::size_t seed     = under(drawn.scene(), *drawn.shown(), 0u);
    REQUIRE(selected != seed);
    check_floor_tones(drawn.scene(), selected);
    CHECK(tone_at(floor_marks(drawn.scene()), seed) == threepp::Color(floor_iterate_tone));
    check_field_tones(drawn.scene());
}

TEST_CASE("the Display window's Iterates switch hides the floor markers", "[optimization][surface][objective][view]")
{
    minimized_surface drawn(elliptic(), descended());
    drawn.select(5u);
    REQUIRE(drawn_visible(drawn.scene(), "iterates on the floor"));

    drawn.press("Iterates");
    CHECK_FALSE(drawn_visible(drawn.scene(), "iterates on the floor"));
    drawn.press("Iterates");
    CHECK(drawn_visible(drawn.scene(), "iterates on the floor"));
}

TEST_CASE("the floor marker of the selected step's iterate wears cyan", "[optimization][surface][objective]")
{
    CHECK(floor_selected_iterate_tone == 0x00ffffu);
}
