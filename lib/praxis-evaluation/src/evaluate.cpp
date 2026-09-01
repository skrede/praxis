#include "praxis/evaluation/slot_evaluation.h"

#include <span>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <string_view>

namespace praxis::evaluation {

namespace {

struct tally
{
    outcome_counts outcomes;
    residual worst;
    std::size_t worst_index;
};

void counted(outcome_counts &seen, agreement verdict)
{
    if(verdict == agreement::agreed)
        ++seen.agreed;
    else if(verdict == agreement::differed)
        ++seen.differed;
    else if(verdict == agreement::one_refused)
        ++seen.one_refused;
    else if(verdict == agreement::both_refused)
        ++seen.both_refused;
    else if(verdict == agreement::refused_differently)
        ++seen.refused_differently;
    else if(verdict == agreement::unusable)
        ++seen.unusable;
}

// Ordered on the magnitude first and on the linear half only where the magnitudes are equal, so the
// worst a run saw never moves backwards as later cases are folded in.
bool worse_than(const residual &seen, const residual &held)
{
    if(seen.magnitude != held.magnitude)
        return seen.magnitude > held.magnitude;

    return seen.linear_error_metres > held.linear_error_metres;
}

void recorded(tally &seen, const case_result &answer, std::size_t index)
{
    counted(seen.outcomes, answer.verdict);
    if(!worse_than(answer.difference, seen.worst))
        return;

    seen.worst       = answer.difference;
    seen.worst_index = index;
}

// A one-sided case is counted and not folded: it says nothing about the two sides taken together, so
// it neither carries a row nor lets one pass. A row none of whose cases had both sides answer
// measured nothing about the pair and is not exercised.
agreement folded(const tally &seen, std::size_t cases)
{
    if(cases == 0)
        return agreement::not_exercised;
    if(seen.outcomes.unusable != 0)
        return agreement::unusable;
    if(seen.outcomes.differed != 0)
        return agreement::differed;
    if(seen.outcomes.refused_differently != 0)
        return agreement::refused_differently;
    if(seen.outcomes.both_refused == cases)
        return agreement::both_refused;
    if(seen.outcomes.agreed == 0)
        return agreement::not_exercised;

    return agreement::agreed;
}

// A row declaring the number of cases its bound was measured over says nothing about a longer run:
// the bound is not known to hold there, so no verdict read at it is claimed. A row declaring none
// carries a bound that does not move with the length of the run and is never held to this.
bool beyond_what_was_measured(const slot_evaluation &slot, std::size_t cases)
{
    return slot.bound_measured_to_cases != 0 && cases > slot.bound_measured_to_cases;
}

slot_report measured(const evaluation_view &compared, const slot_evaluation &slot, std::uint64_t seed, std::size_t cases)
{
    tally seen{outcome_counts{}, residual{slot.kind, 0.0, 0.0}, 0};

    for(std::size_t index = 0; index < cases; ++index)
    {
        case_source drawn = case_source::at_case(seed, slot.name, spread::bulk, index);
        recorded(seen, slot.compare(compared.first(), compared.second(), drawn, slot.allowed), index);
    }

    const agreement reached = beyond_what_was_measured(slot, cases) ? agreement::beyond_measurement : folded(seen, cases);

    return slot_report{compared.extension(), slot.name, reached, seen.worst, cases, seen.worst_index, seen.outcomes};
}

std::vector<std::string_view> described_names(std::span<const capability_view> described)
{
    std::vector<std::string_view> names;

    for(const capability_view &view : described)
        for(const slot_descriptor &slot : view.slots())
            names.push_back(slot.name);

    return names;
}

std::vector<std::string_view> compared_names(std::span<const evaluation_view> compared)
{
    std::vector<std::string_view> names;

    for(const evaluation_view &view : compared)
        for(const slot_evaluation &slot : view.slots())
            names.push_back(slot.name);

    return names;
}

std::vector<std::string_view> missing_from(const std::vector<std::string_view> &wanted, const std::vector<std::string_view> &held)
{
    std::vector<std::string_view> absent;

    for(std::string_view name : wanted)
        if(std::find(held.begin(), held.end(), name) == held.end())
            absent.push_back(name);

    return absent;
}

}

evaluation_report evaluate(std::span<const evaluation_view> compared, std::uint64_t seed, std::size_t cases_per_slot)
{
    evaluation_report reported{{}, seed, cases_per_slot};

    for(const evaluation_view &view : compared)
        for(const slot_evaluation &slot : view.slots())
            reported.slots.push_back(measured(view, slot, seed, cases_per_slot));

    return reported;
}

std::vector<std::string_view> unevaluated_slots(std::span<const capability_view> described, std::span<const evaluation_view> compared)
{
    return missing_from(described_names(described), compared_names(compared));
}

std::vector<std::string_view> unnamed_evaluations(std::span<const capability_view> described, std::span<const evaluation_view> compared)
{
    return missing_from(compared_names(compared), described_names(described));
}

bool every_slot_agreed(const evaluation_report &reported)
{
    for(const slot_report &slot : reported.slots)
        if(slot.verdict != agreement::agreed)
            return false;

    return !reported.slots.empty();
}

std::vector<std::string_view> disagreeing_slots(const evaluation_report &reported)
{
    std::vector<std::string_view> named;

    for(const slot_report &slot : reported.slots)
        if(slot.verdict != agreement::agreed)
            named.push_back(slot.slot);

    return named;
}

}
