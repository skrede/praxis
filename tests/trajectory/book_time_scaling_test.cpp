#include "evaluation_cases.h"
#include "book_polynomial_scalings.h"

#include "praxis/trajectory/evaluation.h"
#include "praxis/trajectory/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/generation.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <cstddef>
#include <string_view>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::evaluation;

namespace {

constexpr std::size_t cubic_row       = 0u;
constexpr std::size_t quintic_row     = 1u;
constexpr std::size_t trapezoidal_row = 2u;

constexpr std::size_t fewest_draws_of_an_instant = 50u;

// Only the time-scaling view is evaluated: it is the first the aggregate lists.
slot_report judged(const trajectory::capabilities &literal, std::size_t row)
{
    const trajectory::capabilities reference = trajectory::baseline();
    const auto views                         = trajectory::evaluation_views(literal, reference);

    return evaluate(std::span(views).first(1), default_seed, default_cases_per_slot).slots.at(row);
}

void require_agreed(const slot_report &row)
{
    REQUIRE(row.verdict == agreement::agreed);
    REQUIRE(row.outcomes.agreed == default_cases_per_slot);
}

using instant_counts = std::array<std::size_t, 5>;

instant_counts snapped_counts(std::string_view slot, trajectory::scaling_case (*drawn_case)(case_source &))
{
    instant_counts counts{};
    for(std::size_t index = 0; index < default_cases_per_slot; ++index)
    {
        case_source drawn                     = case_source::at_case(default_seed, slot, spread::bulk, index);
        const trajectory::scaling_case sample = drawn_case(drawn);
        ++counts.at(static_cast<std::size_t>(sample.snapped));
        if(sample.snapped == trajectory::snapped_instant::start)
            REQUIRE(sample.at == 0.0);
        if(sample.snapped == trajectory::snapped_instant::finish)
            REQUIRE(sample.at == sample.duration);
    }

    return counts;
}

std::size_t drawn_at(const instant_counts &counts, trajectory::snapped_instant instant)
{
    return counts.at(static_cast<std::size_t>(instant));
}

}

TEST_CASE("every_literal_cubic_is_judged_agreeing_by_the_cubic_row")
{
    const auto cubics = book_cubics();
    for(std::size_t form = 0; form < cubics.size(); ++form)
    {
        trajectory::capabilities literal = trajectory::baseline();
        literal.time_scaling.cubic       = cubics.at(form);
        const slot_report row            = judged(literal, cubic_row);

        INFO("form " << form);
        REQUIRE(row.slot == "time_scaling.cubic");
        require_agreed(row);
    }
}

TEST_CASE("every_literal_quintic_is_judged_agreeing_by_the_quintic_row")
{
    const auto quintics = book_quintics();
    for(std::size_t form = 0; form < quintics.size(); ++form)
    {
        trajectory::capabilities literal = trajectory::baseline();
        literal.time_scaling.quintic     = quintics.at(form);
        const slot_report row            = judged(literal, quintic_row);

        INFO("form " << form);
        REQUIRE(row.slot == "time_scaling.quintic");
        require_agreed(row);
    }
}

TEST_CASE("every_literal_trapezoid_is_judged_agreeing_by_the_trapezoidal_row")
{
    const auto trapezoids = book_trapezoids();
    for(std::size_t form = 0; form < trapezoids.size(); ++form)
    {
        trajectory::capabilities literal = trajectory::baseline();
        literal.time_scaling.trapezoidal = trapezoids.at(form);
        const slot_report row            = judged(literal, trapezoidal_row);

        INFO("form " << form);
        REQUIRE(row.slot == "time_scaling.trapezoidal");
        require_agreed(row);
    }
}

TEST_CASE("each_instant_a_scaling_row_snaps_to_is_drawn_at_least_fifty_times_at_the_default_draw")
{
    using trajectory::snapped_instant;

    for(const std::string_view slot : {"time_scaling.cubic", "time_scaling.quintic"})
    {
        const instant_counts counts = snapped_counts(slot, &trajectory::drawn_scaling_case);

        INFO(slot);
        REQUIRE(drawn_at(counts, snapped_instant::start) >= fewest_draws_of_an_instant);
        REQUIRE(drawn_at(counts, snapped_instant::finish) >= fewest_draws_of_an_instant);
        REQUIRE(drawn_at(counts, snapped_instant::ramp_end) + drawn_at(counts, snapped_instant::coast_end) == 0u);
    }

    const instant_counts counts = snapped_counts("time_scaling.trapezoidal", &trajectory::drawn_trapezoid_case);
    for(const snapped_instant instant : {snapped_instant::start, snapped_instant::ramp_end, snapped_instant::coast_end, snapped_instant::finish})
        REQUIRE(drawn_at(counts, instant) >= fewest_draws_of_an_instant);
}

TEST_CASE("the_trapezoidal_row_draws_no_duration_beneath_the_floor_its_bounds_set")
{
    for(std::size_t index = 0; index < default_cases_per_slot; ++index)
    {
        case_source drawn                     = case_source::at_case(default_seed, "time_scaling.trapezoidal", spread::bulk, index);
        const trajectory::scaling_case sample = trajectory::drawn_trapezoid_case(drawn);

        REQUIRE(sample.duration >= trajectory::trapezoid_duration_floor(sample.speed_bound, sample.acceleration_bound));
        REQUIRE(sample.at >= 0.0);
        REQUIRE(sample.at <= sample.duration);
    }
}
