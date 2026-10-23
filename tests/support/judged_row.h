#ifndef HPP_GUARD_PRAXIS_TESTS_SUPPORT_JUDGED_ROW_H
#define HPP_GUARD_PRAXIS_TESTS_SUPPORT_JUDGED_ROW_H

#include "praxis/optimization/evaluation.h"
#include "praxis/optimization/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <cstdint>
#include <algorithm>
#include <string_view>

namespace praxis::tests {

// Only the view holding the named row is evaluated, and its report is found by that name.
inline evaluation::slot_report judged(const optimization::capabilities &first, const optimization::capabilities &second, std::string_view name, std::uint64_t seed)
{
    const std::array<evaluation::evaluation_view, 2> views = optimization::evaluation_views(first, second);
    const auto holding                                     = [name](const evaluation::evaluation_view &view)
    { return std::any_of(view.slots().begin(), view.slots().end(), [name](const evaluation::slot_evaluation &row) { return row.name == name; }); };
    const auto view = std::find_if(views.begin(), views.end(), holding);
    REQUIRE(view != views.end());

    const evaluation::evaluation_report reported = evaluation::evaluate(std::span(view, 1), seed, evaluation::default_cases_per_slot);
    const auto row                               = std::find_if(reported.slots.begin(), reported.slots.end(), [name](const evaluation::slot_report &r) { return r.slot == name; });
    REQUIRE(row != reported.slots.end());
    return *row;
}

}

#endif
