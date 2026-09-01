#include "evaluation_cases.h"
#include "evaluation_tables.h"

#include "praxis/trajectory/slots.h"

#include "praxis/evaluation/comparators.h"

#include <Eigen/Core>

#include <array>
#include <cstddef>

namespace praxis::trajectory {

namespace {

const time_scaling_ops &time_scalings_of(const void *value)
{
    return *static_cast<const time_scaling_ops *>(value);
}

// The path parameter and both its derivatives read as one column, so a single element-wise residual
// covers the whole sample rather than the parameter alone.
Eigen::Vector3d sampled(const scaling_sample &point)
{
    return Eigen::Vector3d(point.s, point.ds, point.dds);
}

evaluation::residual between(const scaling_sample &held, const scaling_sample &against)
{
    return evaluation::element_wise_residual(sampled(held), sampled(against));
}

evaluation::case_result compared(const expected<scaling_sample, refusal> &held, const expected<scaling_sample, refusal> &against, const evaluation::tolerance_pair &allowed)
{
    return evaluation::agreed_or_refused(held, against, between, allowed);
}

evaluation::case_result compare_cubic(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const scaling_case example = drawn_scaling_case(drawn);

    return compared(time_scalings_of(first).cubic(example.at, example.duration), time_scalings_of(second).cubic(example.at, example.duration), allowed);
}

evaluation::case_result compare_quintic(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const scaling_case example = drawn_scaling_case(drawn);

    return compared(time_scalings_of(first).quintic(example.at, example.duration), time_scalings_of(second).quintic(example.at, example.duration), allowed);
}

evaluation::case_result compare_trapezoidal(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const scaling_case example = drawn_scaling_case(drawn);

    return compared(time_scalings_of(first).trapezoidal(example.at, example.duration, example.speed_bound, example.acceleration_bound),
                    time_scalings_of(second).trapezoidal(example.at, example.duration, example.speed_bound, example.acceleration_bound), allowed);
}

constexpr evaluation::residual_kind sample_kind = evaluation::residual_kind::element_wise;

// The rows are in the enumerator order of time_scaling_slot, and each name is spelled exactly as the
// descriptor table spells it. Every slot this aggregate describes is compared here, which is what
// the assertion below the table holds.
constexpr evaluation::tolerance_pair sample_allowance{scaling_sample_tolerance, scaling_sample_tolerance};
constexpr evaluation::tolerance_pair quintic_allowance{quintic_scaling_tolerance, quintic_scaling_tolerance};

constexpr std::array time_scaling_table{
        evaluation::slot_evaluation{"time_scaling.cubic", sample_kind, sample_allowance, &compare_cubic},
        evaluation::slot_evaluation{"time_scaling.quintic", sample_kind, quintic_allowance, &compare_quintic},
        evaluation::slot_evaluation{"time_scaling.trapezoidal", sample_kind, sample_allowance, &compare_trapezoidal},
};

static_assert(time_scaling_table.size() == static_cast<std::size_t>(time_scaling_slot::count));

constexpr evaluation::capability_evaluations<time_scaling_ops> evaluated_time_scalings{"trajectory", time_scaling_table};

}

const evaluation::capability_evaluations<time_scaling_ops> &time_scaling_evaluations()
{
    return evaluated_time_scalings;
}

}
