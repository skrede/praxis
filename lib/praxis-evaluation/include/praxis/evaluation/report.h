#ifndef HPP_GUARD_PRAXIS_EVALUATION_REPORT_H
#define HPP_GUARD_PRAXIS_EVALUATION_REPORT_H

#include "praxis/evaluation/residual.h"

#include <vector>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace praxis::evaluation {

// What `evaluate` draws with where a caller passes nothing of its own.
inline constexpr std::uint64_t default_seed         = 0x5EEDu;
inline constexpr std::size_t default_cases_per_slot = 1000;

// How many of a run's cases reached each outcome. The six sum to the run's `cases`: every case
// reaches exactly one of them, and `not_exercised` and `beyond_measurement` are verdicts over a run
// rather than outcomes a case can have.
struct outcome_counts
{
    std::size_t agreed;
    std::size_t differed;
    std::size_t one_refused;
    std::size_t both_refused;
    std::size_t refused_differently;
    std::size_t unusable;
};

// `worst` is the largest residual any of the `cases` produced and `worst_case_index` is where in the
// run it came up, counting from zero; both stand at the kind's own zero when no case was run.
// Declaration order is frozen: a table written as a brace initializer changes meaning if a member
// moves. Appending is safe.
struct slot_report
{
    std::string_view extension;
    std::string_view slot;
    agreement verdict;
    residual worst;
    std::size_t cases;
    std::size_t worst_case_index;
    outcome_counts outcomes;
};

struct evaluation_report
{
    std::vector<slot_report> slots;
    std::uint64_t seed;
    std::size_t cases_per_slot;
};

// False for a report carrying no slot at all, and false for a slot no case exercised, so a run that
// measured nothing is never a pass. A slot none of whose cases had both sides answer measured
// nothing either, and is reported as not exercised rather than as agreement.
bool every_slot_agreed(const evaluation_report &reported);

// Every slot the run did not record as agreed, in the report's own order: one that differed, one
// whose two sides refused apart, one whose input the harness could not build, and one that was never
// exercised are all named here.
std::vector<std::string_view> disagreeing_slots(const evaluation_report &reported);

}

#endif
