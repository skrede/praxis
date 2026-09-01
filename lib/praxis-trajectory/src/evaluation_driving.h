#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_EVALUATION_DRIVING_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_EVALUATION_DRIVING_H

#include "praxis/trajectory/evaluation.h"
#include "praxis/trajectory/trajectory.h"
#include "praxis/trajectory/pose_trajectory.h"

#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/slot_evaluation.h"

#include "praxis/compat/expected.h"

#include <limits>
#include <memory>
#include <vector>
#include <cstddef>
#include <optional>

namespace praxis::trajectory {

inline evaluation::case_result judged(const evaluation::residual &difference, const evaluation::tolerance_pair &allowed)
{
    return evaluation::case_result{evaluation::verdict_of(difference, allowed), difference};
}

// Two answers with nothing in common to measure over: the magnitude stands beyond every bound rather
// than at a large number standing in for one.
inline evaluation::case_result differing(evaluation::residual_kind kind)
{
    constexpr double unbounded = std::numeric_limits<double>::infinity();

    return evaluation::case_result{evaluation::agreement::differed, evaluation::residual{kind, unbounded, unbounded}};
}

// The harness's own outcome: the shared input could not be built, so neither side was asked about it
// and neither is answerable for it.
inline evaluation::case_result unusable_input(evaluation::residual_kind kind)
{
    constexpr double unbounded = std::numeric_limits<double>::infinity();

    return evaluation::case_result{evaluation::agreement::unusable, evaluation::residual{kind, unbounded, unbounded}};
}

// The facility's refusal policy, written out for the shape its refusal-aware helper cannot be
// instantiated over: an answer here is a prepared object, and what follows drives it rather than
// subtracting it. Absent where both sides answered, so no pointer is read until this has run.
template<typename T>
std::optional<evaluation::case_result> refusal_outcome(const expected<T, refusal> &first, const expected<T, refusal> &second)
{
    if(first.has_value() && second.has_value())
        return std::nullopt;
    if(first.has_value() != second.has_value())
        return evaluation::case_result{evaluation::agreement::one_refused, evaluation::residual{}};

    return evaluation::case_result{first.error() == second.error() ? evaluation::agreement::both_refused : evaluation::agreement::refused_differently, evaluation::residual{}};
}

// A factory that answered a null pointer named no motion, so there is nothing to drive and nothing
// to compare; the pair is reported as differing rather than read through a pointer nobody promised.
template<typename Generator>
bool both_named_a_motion(const std::unique_ptr<Generator> &held, const std::unique_ptr<Generator> &other)
{
    return held != nullptr && other != nullptr;
}

// A run standing still at a row traverses that row in no time.
template<typename Row>
bool advances_at_every_row(const Row &seed, const std::vector<Row> &waypoints)
{
    const Row *before = &seed;

    for(const Row &row : waypoints)
    {
        if(row == *before)
            return false;

        before = &row;
    }

    return true;
}

}

#endif
