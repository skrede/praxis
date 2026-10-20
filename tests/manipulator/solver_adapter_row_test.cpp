#include "praxis/manipulator/evaluation.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/solver_adapters.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <cstdint>
#include <algorithm>

using namespace praxis;
using namespace praxis::manipulator;
using namespace praxis::evaluation;

namespace {

using adapter = decltype(inverse_kinematics_ops::inverse_kinematics);

bool carries_the_inverse_kinematics_row(const evaluation_view &view)
{
    return std::ranges::any_of(view.slots(), [](const slot_evaluation &slot) { return slot.name == "ik.inverse_kinematics"; });
}

slot_report judged(adapter solve, std::uint64_t seed)
{
    const capabilities reference               = manipulator::baseline();
    capabilities bound                         = manipulator::baseline();
    bound.ik.inverse_kinematics                = solve;
    const std::array<evaluation_view, 7> views = evaluation_views(reference, bound);
    const auto row                             = std::ranges::find_if(views, &carries_the_inverse_kinematics_row);
    REQUIRE(row != views.end());

    return evaluate(std::span(row, 1), seed, default_cases_per_slot).slots.at(0);
}

void passes_the_row(adapter solve)
{
    for(std::uint64_t seed = default_seed; seed <= default_seed + 40u; ++seed)
    {
        const slot_report row = judged(solve, seed);

        INFO("seed " << seed);
        REQUIRE(row.slot == "ik.inverse_kinematics");
        REQUIRE(row.cases == default_cases_per_slot);
        CHECK(row.outcomes.differed + row.outcomes.refused_differently + row.outcomes.unusable == 0u);
        CHECK(row.outcomes.agreed > 0u);
        CHECK(row.verdict == agreement::agreed);
    }
}

}

TEST_CASE("the body adapter bound into the inverse kinematics slot passes the inverse kinematics row")
{
    passes_the_row(&body_inverse_kinematics);
}

TEST_CASE("the space adapter bound into the inverse kinematics slot passes the inverse kinematics row")
{
    passes_the_row(&space_inverse_kinematics);
}
