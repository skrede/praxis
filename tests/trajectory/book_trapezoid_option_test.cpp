#include "evaluation_cases.h"
#include "book_trapezoid_options.h"

#include "praxis/trajectory/evaluation.h"
#include "praxis/trajectory/capabilities.h"
#include "praxis/trajectory/baseline/time_scaling.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/generation.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::evaluation;

namespace {

using trajectory::scaling_case;
using trajectory::snapped_instant;
using trajectory::time_scaling_ops;

using option_member = option_slot time_scaling_ops::*;
using drawn_case    = scaling_case (*)(case_source &);

struct option_row
{
    std::string_view slot;
    std::size_t row;
    option_member member;
    drawn_case draw;
};

constexpr std::array option_rows{
        option_row{"time_scaling.trapezoidal_from_velocity_acceleration", 2u, &time_scaling_ops::trapezoidal_from_velocity_acceleration, &trajectory::drawn_velocity_acceleration_case},
        option_row{"time_scaling.trapezoidal_from_velocity_duration", 3u, &time_scaling_ops::trapezoidal_from_velocity_duration, &trajectory::drawn_velocity_duration_case},
        option_row{"time_scaling.trapezoidal_from_acceleration_duration", 4u, &time_scaling_ops::trapezoidal_from_acceleration_duration, &trajectory::drawn_acceleration_duration_case},
};

constexpr std::size_t later_seeds                = 40u;
constexpr std::size_t fewest_draws_of_an_instant = 50u;

// Only the time-scaling view is evaluated: it is the first the aggregate lists.
slot_report judged(const option_row &option, option_slot literal, std::uint64_t seed)
{
    trajectory::capabilities transcribed      = trajectory::baseline();
    transcribed.time_scaling.*(option.member) = literal;

    const trajectory::capabilities reference = trajectory::baseline();
    const auto views                         = trajectory::evaluation_views(transcribed, reference);

    return evaluate(std::span(views).first(1), seed, default_cases_per_slot).slots.at(option.row);
}

void require_population_agreed(const option_row &option, std::span<const option_slot> population)
{
    for(std::size_t form = 0; form < population.size(); ++form)
    {
        const slot_report row = judged(option, population[form], default_seed);

        INFO(option.slot << " form " << form);
        REQUIRE(row.slot == option.slot);
        REQUIRE(row.verdict == agreement::agreed);
        REQUIRE(row.outcomes.agreed == default_cases_per_slot);
    }
}

// The printed spelling with no domain check, in every phase shape.
void require_printed_agreed_at_later_seeds(const option_row &option, std::span<const option_slot> population)
{
    for(std::size_t form = 0; form < option_shape_count; ++form)
    {
        if(option_shape_at(form).check != domain_check::none)
            continue;
        for(std::uint64_t seed = default_seed + 1u; seed <= default_seed + later_seeds; ++seed)
        {
            INFO(option.slot << " form " << form << " seed " << seed);
            REQUIRE(judged(option, population[form], seed).verdict == agreement::agreed);
        }
    }
}

// The two inputs after t the row hands its slot: (v, a), (T, v) or (T, a).
std::array<double, 2> handed(const option_row &option, const scaling_case &sample)
{
    if(option.row == 2u)
        return {sample.speed_bound, sample.acceleration_bound};
    if(option.row == 3u)
        return {sample.duration, sample.speed_bound};

    return {sample.duration, sample.acceleration_bound};
}

bool in_domain(const option_row &option, const scaling_case &sample)
{
    const double v = sample.speed_bound;
    const double a = sample.acceleration_bound;
    const double T = sample.duration;
    if(option.row == 2u)
        return v * v / a <= 1.0;
    if(option.row == 3u)
        return v * T > 1.0 && v * T <= 2.0;

    return a * T * T >= 4.0;
}

}

TEST_CASE("every_literal_choosing_v_and_a_is_judged_agreeing_by_its_row")
{
    require_population_agreed(option_rows[0], book_velocity_acceleration_options());
    require_printed_agreed_at_later_seeds(option_rows[0], book_velocity_acceleration_options());
}

TEST_CASE("every_literal_choosing_v_and_t_is_judged_agreeing_by_its_row")
{
    require_population_agreed(option_rows[1], book_velocity_duration_options());
    require_printed_agreed_at_later_seeds(option_rows[1], book_velocity_duration_options());
}

TEST_CASE("every_literal_choosing_a_and_t_is_judged_agreeing_by_its_row")
{
    require_population_agreed(option_rows[2], book_acceleration_duration_options());
    require_printed_agreed_at_later_seeds(option_rows[2], book_acceleration_duration_options());
}

TEST_CASE("each_instant_a_book_trapezoid_row_snaps_to_is_drawn_at_least_fifty_times_at_the_default_draw")
{
    for(const option_row &option : option_rows)
    {
        std::array<std::size_t, 5> counts{};
        for(std::size_t index = 0; index < default_cases_per_slot; ++index)
        {
            case_source drawn = case_source::at_case(default_seed, option.slot, spread::bulk, index);
            ++counts.at(static_cast<std::size_t>(option.draw(drawn).snapped));
        }

        INFO(option.slot);
        for(const snapped_instant instant : {snapped_instant::start, snapped_instant::ramp_end, snapped_instant::coast_end, snapped_instant::finish})
            REQUIRE(counts.at(static_cast<std::size_t>(instant)) >= fewest_draws_of_an_instant);
    }
}

TEST_CASE("every_case_a_book_trapezoid_row_draws_lies_in_its_domain_and_the_reference_answers_it")
{
    const time_scaling_ops reference = trajectory::baseline().time_scaling;
    for(const option_row &option : option_rows)
    {
        for(std::size_t index = 0; index < default_cases_per_slot; ++index)
        {
            case_source drawn          = case_source::at_case(default_seed, option.slot, spread::bulk, index);
            const scaling_case sample  = option.draw(drawn);
            const auto [first, second] = handed(option, sample);

            INFO(option.slot << " case " << index);
            REQUIRE(in_domain(option, sample));
            REQUIRE(sample.at >= 0.0);
            REQUIRE(sample.at <= sample.duration);
            REQUIRE((reference.*(option.member))(sample.at, first, second).has_value());
        }
    }
}

// Lynch & Park, Modern Robotics, sec. 9.2.2.2: where v^2/a approaches 1 the coast all but vanishes.
TEST_CASE("at_the_upper_edge_of_the_v_and_a_draw_every_literal_steps_its_second_derivative_to_a_side_its_row_accepts")
{
    const auto population = book_velocity_acceleration_options();
    for(std::size_t step = 0; step <= 95u; ++step)
    {
        const double a        = 0.5 + 9.5 * static_cast<double>(step) / 95.0;
        const double v        = std::sqrt(a) * (1.0 - 1.0e-13);
        const double duration = (a + v * v) / (v * a);
        const double ramp_end = v / a;

        for(std::size_t form = 0; form < population.size(); ++form)
        {
            INFO("a " << a << " form " << form);
            const double at_ramp_end  = population[form](ramp_end, v, a)->dds;
            const double at_coast_end = population[form](duration - ramp_end, v, a)->dds;
            REQUIRE((at_ramp_end == a || at_ramp_end == 0.0));
            REQUIRE((at_coast_end == 0.0 || at_coast_end == -a));
        }
    }
}
