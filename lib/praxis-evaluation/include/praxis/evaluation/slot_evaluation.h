#ifndef HPP_GUARD_PRAXIS_EVALUATION_SLOT_EVALUATION_H
#define HPP_GUARD_PRAXIS_EVALUATION_SLOT_EVALUATION_H

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/generation.h"

#include "praxis/extension/refusal.h"
#include "praxis/extension/descriptor.h"

#include "praxis/compat/expected.h"

#include <span>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace praxis::evaluation {

struct case_result
{
    agreement verdict;
    residual difference;
};

// Declaration order is frozen: every table is written as a brace initializer, so reordering a member
// silently changes what an existing table means. Appending is safe. `name` is the slot's name
// spelled exactly as the describing table spells it -- the two cross-checks below compare the two
// name sets byte for byte, so a difference in case or in whitespace is a mismatch.
// `bound_measured_to_cases` is how many cases `allowed` was measured over, and so the longest run it
// is known to hold over. Zero declares no such limit and is what a row leaving the member out means;
// a row whose bound does not move with the length of the run leaves it out.
struct slot_evaluation
{
    std::string_view name;
    residual_kind kind;
    tolerance_pair allowed;
    case_result (*compare)(const void *first, const void *second, case_source &drawn, const tolerance_pair &allowed);
    std::size_t bound_measured_to_cases = 0;
};

template<typename Ops>
struct capability_evaluations
{
    std::string_view extension;
    std::span<const slot_evaluation> slots;
};

class evaluation_view
{
public:
    evaluation_view()
            : m_first(nullptr)
            , m_second(nullptr)
            , m_extension()
            , m_slots()
    {
    }

    // Ops is deduced from all three arguments, so a pair described by another capability's table is
    // a deduction conflict rather than a comparator casting the pointers to a type nobody checked.
    template<typename Ops>
    static evaluation_view of(const Ops &first, const Ops &second, const capability_evaluations<Ops> &described)
    {
        return evaluation_view(&first, &second, described.extension, described.slots);
    }

    // A view points into the two values it was given, which must outlive it. A temporary argument
    // would leave the view dangling at the end of the full expression, so those calls are deleted
    // rather than diagnosed at run time.
    template<typename Ops>
    static evaluation_view of(Ops &&, const Ops &, const capability_evaluations<Ops> &) = delete;

    template<typename Ops>
    static evaluation_view of(const Ops &, Ops &&, const capability_evaluations<Ops> &) = delete;

    const void *first() const
    {
        return m_first;
    }

    const void *second() const
    {
        return m_second;
    }

    std::string_view extension() const
    {
        return m_extension;
    }

    std::span<const slot_evaluation> slots() const
    {
        return m_slots;
    }

private:
    const void *m_first;
    const void *m_second;
    std::string_view m_extension;
    std::span<const slot_evaluation> m_slots;

    evaluation_view(const void *first, const void *second, std::string_view extension, std::span<const slot_evaluation> slots)
            : m_first(first)
            , m_second(second)
            , m_extension(extension)
            , m_slots(slots)
    {
    }
};

// The outcome of one comparison whose two sides may each decline the input. Where both answered,
// the residual is `compared(first.value(), second.value())` and the verdict is `verdict_of` over it.
// Where both declined for the same reason the verdict is `both_refused`: two bindings that decline
// the same input agree about that input. Where they declined for different reasons it is
// `refused_differently`, an outcome of its own rather than one folded into agreement or
// disagreement, since praxis does not own the question of whether a consumer cares which reason a
// refusal carried. Where exactly one declined it is `one_refused`. Every outcome but the first
// carries a value-initialized residual: both halves are exactly zero and the kind is not read,
// because a refusal is never turned into a number and never into a large one standing in for
// infinity.
template<typename T, typename Compare>
case_result agreed_or_refused(const expected<T, refusal> &first, const expected<T, refusal> &second, Compare compared, const tolerance_pair &allowed)
{
    if(first.has_value() && second.has_value())
    {
        const residual difference = compared(first.value(), second.value());

        return case_result{verdict_of(difference, allowed), difference};
    }
    if(first.has_value() != second.has_value())
        return case_result{agreement::one_refused, residual{}};

    return case_result{first.error() == second.error() ? agreement::both_refused : agreement::refused_differently, residual{}};
}

// Every view in span order, and within a view every slot in its table's order, which is the
// capability's own enumerator order. A slot asked for more cases than its bound was measured over
// reports `beyond_measurement` and no verdict of its own, whatever its cases reached; its counts and
// its worst residual are recorded as for any other run. Otherwise a slot's verdict is `unusable` if
// any case was unusable; otherwise `differed` if any case differed; otherwise `refused_differently`
// if any case had both sides refuse for different reasons; otherwise `both_refused` if every case had
// both sides refuse; otherwise `agreed`. A run of zero cases, and a run no case of which had both
// sides answer, report
// `not_exercised` and never `agreed`: a case where exactly one side refused is carried as a count on
// the slot's report and folds nothing. Every case's outcome is counted there whatever the verdict.
evaluation_report evaluate(std::span<const evaluation_view> compared, std::uint64_t seed = default_seed, std::size_t cases_per_slot = default_cases_per_slot);

// The two halves of the ledger, each total: an empty span answers an empty vector. The first names
// every described slot no table compares, the second every compared slot no describing table names.
std::vector<std::string_view> unevaluated_slots(std::span<const capability_view> described, std::span<const evaluation_view> compared);
std::vector<std::string_view> unnamed_evaluations(std::span<const capability_view> described, std::span<const evaluation_view> compared);

}

#endif
