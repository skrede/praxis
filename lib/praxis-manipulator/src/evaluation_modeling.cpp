#include "evaluation_tables.h"

#include "praxis/manipulator/slots.h"

#include <array>
#include <cstddef>

namespace praxis::manipulator {

namespace {

// The single row is in the enumerator order of modeling_slot and its name is spelled exactly as the
// descriptor table spells it. Its comparator sits in evaluation_chain.cpp beside the two others the
// shipped residual vocabulary could not serve, since a chain is judged by what it computes rather
// than by subtracting its members.
constexpr std::array modeling_table{
        evaluation::slot_evaluation{"modeling.build_chain", evaluation::residual_kind::pose, evaluation::tolerance_of(evaluation::residual_kind::pose), &compare_build_chain},
};

static_assert(modeling_table.size() == static_cast<std::size_t>(modeling_slot::count));

constexpr evaluation::capability_evaluations<modeling_ops> evaluated_modelings{"manipulator", modeling_table};

}

const evaluation::capability_evaluations<modeling_ops> &modeling_evaluations()
{
    return evaluated_modelings;
}

}
