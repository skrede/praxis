#include "bent_manipulator.h"
#include "evaluation_cases.h"
#include "described_slots.h"

#include "praxis/manipulator/slots.h"
#include "praxis/manipulator/evaluation.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/generation.h"
#include "praxis/evaluation/slot_evaluation.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <meios/model.h>

#include <span>
#include <array>
#include <vector>
#include <string>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <string_view>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

constexpr std::uint64_t recorded_seed = 0xC0FFEEu;
constexpr std::size_t cases_per_row   = 8u;
constexpr std::size_t drawn_examples  = 16u;

using fixture::compared_slots;

// A millimetre and a milliradian, nine decades and more above every bound the tables carry.
constexpr std::array<double, static_cast<std::size_t>(fixture::bent::count)> plainly_wrong{1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3};

std::size_t carrying(std::span<const evaluation_view> compared, residual_kind kind)
{
    std::size_t seen = 0;

    for(const evaluation_view &view : compared)
        for(const slot_evaluation &slot : view.slots())
            seen += slot.kind == kind ? 1u : 0u;

    return seen;
}

std::string joined(const std::vector<std::string_view> &reported)
{
    std::string listed;

    for(std::string_view slot : reported)
        listed += (listed.empty() ? "" : ", ") + std::string(slot);

    return listed.empty() ? std::string("none") : listed;
}

bool alike(const evaluation_report &reported, const evaluation_report &again)
{
    if(reported.slots.size() != again.slots.size())
        return false;

    for(std::size_t index = 0; index < reported.slots.size(); ++index)
        if(reported.slots[index].slot != again.slots[index].slot || reported.slots[index].verdict != again.slots[index].verdict ||
           reported.slots[index].worst.magnitude != again.slots[index].worst.magnitude)
            return false;

    return true;
}

}

TEST_CASE("every_row_names_a_slot_one_of_the_describing_tables_declares")
{
    const manipulator::capabilities reference = manipulator::baseline();
    const auto described                      = manipulator::capability_views(reference);
    const auto compared                       = manipulator::evaluation_views(reference, reference);

    REQUIRE(unnamed_evaluations(described, compared).empty());
}

TEST_CASE("every_compared_slot_is_described")
{
    const manipulator::capabilities reference = manipulator::baseline();
    const auto described                      = manipulator::capability_views(reference);
    const auto compared                       = manipulator::evaluation_views(reference, reference);

    const std::vector<std::string_view> without_a_name = unnamed_evaluations(described, compared);

    INFO("compared but never described: " << joined(without_a_name));
    REQUIRE(without_a_name.empty());
}

// A described slot no table compares is named rather than passing unseen, which is the whole of what
// the tables are allowed to be narrower than the descriptions for.
TEST_CASE("the_closed_form_slot_is_described_and_no_table_compares_it")
{
    const manipulator::capabilities reference = manipulator::baseline();
    const auto described                      = manipulator::capability_views(reference);
    const auto compared                       = manipulator::evaluation_views(reference, reference);

    const std::vector<std::string_view> without_a_row = unevaluated_slots(described, compared);

    INFO("described but never compared: " << joined(without_a_row));
    REQUIRE(without_a_row.size() == 1u);
    REQUIRE(without_a_row.front() == "ik.analytic_inverse_kinematics");
}

TEST_CASE("the_reference_answers_every_row_as_it_answers_it_and_the_run_repeats")
{
    fixture::bend_every_row_by({});

    const manipulator::capabilities reference = manipulator::baseline();
    const auto compared                       = manipulator::evaluation_views(reference, reference);
    const evaluation_report reported          = evaluate(compared, recorded_seed, cases_per_row);
    const evaluation_report again             = evaluate(compared, recorded_seed, cases_per_row);

    REQUIRE(reported.slots.size() == compared_slots);
    for(const slot_report &slot : reported.slots)
    {
        INFO("slot " << slot.slot);
        REQUIRE((slot.verdict == agreement::agreed || slot.verdict == agreement::both_refused));
    }
    REQUIRE(alike(reported, again));
}

TEST_CASE("each_rows_residual_kind_is_the_one_its_return_type_calls_for")
{
    const manipulator::capabilities reference = manipulator::baseline();
    const auto compared                       = manipulator::evaluation_views(reference, reference);

    REQUIRE(carrying(compared, residual_kind::pose) == 11u);
    REQUIRE(carrying(compared, residual_kind::element_wise) == 5u);
    REQUIRE(carrying(compared, residual_kind::geodesic) == 1u);
    REQUIRE(carrying(compared, residual_kind::axis_up_to_sign) == 0u);
    REQUIRE(carrying(compared, residual_kind::log_up_to_branch) == 0u);
}

TEST_CASE("every_row_reports_a_difference_against_a_binding_displaced_beyond_its_tolerance")
{
    fixture::bend_every_row_by(plainly_wrong);

    const manipulator::capabilities reference = manipulator::baseline();
    const manipulator::capabilities bent      = fixture::bent_everywhere();
    const auto compared                       = manipulator::evaluation_views(reference, bent);
    const evaluation_report reported          = evaluate(compared, recorded_seed, cases_per_row);

    fixture::bend_every_row_by({});

    REQUIRE(reported.slots.size() == compared_slots);
    for(const slot_report &slot : reported.slots)
    {
        INFO("slot " << slot.slot);
        REQUIRE(slot.verdict == agreement::differed);
        REQUIRE(slot.worst.magnitude + slot.worst.linear_error_metres > 0.0);
    }
}

TEST_CASE("a_drawn_case_and_a_drawn_machine_carry_one_entry_per_axis_the_source_drew")
{
    const manipulator::capabilities reference = manipulator::baseline();

    for(std::size_t index = 0; index < drawn_examples; ++index)
    {
        case_source drawn                          = case_source::at_case(recorded_seed, "modeling.build_chain", spread::bulk, index);
        const manipulator::evaluation_case example = manipulator::drawn_case(drawn);
        const meios::model<> machine               = manipulator::drawn_model(drawn);

        REQUIRE(example.chain.joint_count() >= 1u);
        REQUIRE(static_cast<std::size_t>(example.joints.size()) == example.chain.joint_count());
        REQUIRE(static_cast<std::size_t>(example.chain.limits.velocity.size()) == example.chain.joint_count());
        REQUIRE(static_cast<std::size_t>(example.chain.limits.lower_position.size()) == example.chain.joint_count());
        REQUIRE(manipulator::kinematics::compose(example.chain, reference.fk, reference.dk, reference.ik, rigid_motion::baseline().screw, rigid_motion::baseline().frame).has_value());
        REQUIRE(machine.links.size() == machine.joints.size() + 1u);
        REQUIRE(machine.topo.parent_of.size() == machine.links.size());
    }
}
