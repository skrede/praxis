#include "drawn_surface.h"
#include "minimized_surface.h"

#include "praxis/optimization/scenario_state.h"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>
#include <utility>

using namespace praxis;
using namespace praxis::fixture;

TEST_CASE("over a minimized surface the Display window switches the level curves, the gradient and the step", "[optimization][surface][objective][view]")
{
    minimized_surface drawn(problem_form::minimum);
    threepp::Scene &scene = drawn.scene();

    const std::string offered = drawn.offered();
    for(const char *label : {"Level curves", "Gradient", "Step"})
        CHECK(offered.find(label) != std::string::npos);
    CHECK(offered.find("Zero curve") == std::string::npos);

    const std::vector<std::pair<const char *, const char *>> switched{{"Level curves", "level curves"}, {"Gradient", "gradient at x_k"}, {"Step", "step on the floor"}};
    for(const auto &[label, name] : switched)
    {
        REQUIRE(drawn_visible(scene, name));
        drawn.press(label);
        CHECK_FALSE(drawn_visible(scene, name));
        for(const auto &[other_label, other] : switched)
            if(other != name)
                CHECK(drawn_visible(scene, other));
        drawn.press(label);
        CHECK(drawn_visible(scene, name));
    }
}

TEST_CASE("over a root-form surface the Display window switches the zero curve and offers no level curves or gradient", "[optimization][surface][objective][view]")
{
    minimized_surface drawn(problem_form::root);

    const std::string offered = drawn.offered();
    CHECK(offered.find("Zero curve") != std::string::npos);
    CHECK(offered.find("Level curves") == std::string::npos);
    CHECK(offered.find("Gradient") == std::string::npos);
}
