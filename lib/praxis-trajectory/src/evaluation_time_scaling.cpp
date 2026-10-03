#include "evaluation_cases.h"
#include "evaluation_tables.h"

#include "praxis/trajectory/slots.h"

#include "praxis/evaluation/comparators.h"

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <limits>
#include <cstddef>
#include <algorithm>

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

// The second derivative on either side of an instant where it steps, a side repeated where an
// instant has two.
using sides = std::array<double, 3>;

double nearest_side(double value, const sides &around)
{
    return std::min({std::abs(value - around[0]), std::abs(value - around[1]), std::abs(value - around[2])});
}

double stepped(double held, double against, const sides &around)
{
    if(!std::isfinite(held) || !std::isfinite(against))
        return std::numeric_limits<double>::infinity();

    return std::min(std::abs(held - against), std::max(nearest_side(held, around), nearest_side(against, around)));
}

// s and ds element-wise, and dds the smaller of the two answers' difference and the farther of them
// from its nearest side.
evaluation::residual kinked(const scaling_sample &held, const scaling_sample &against, const sides &around)
{
    return evaluation::element_wise_residual(Eigen::Vector3d(held.s, held.ds, stepped(held.dds, against.dds, around)), Eigen::Vector3d(against.s, against.ds, 0.0));
}

evaluation::case_result compared(const expected<scaling_sample, refusal> &held, const expected<scaling_sample, refusal> &against, const evaluation::tolerance_pair &allowed,
                                 const scaling_case &example, const sides &around)
{
    if(example.snapped == snapped_instant::drawn)
        return evaluation::agreed_or_refused(held, against, between, allowed);

    return evaluation::agreed_or_refused(held, against, [&around](const scaling_sample &first, const scaling_sample &second) { return kinked(first, second, around); }, allowed);
}

// `opening` is the second derivative a polynomial scaling starts at and the negation of the one it
// finishes at.
sides polynomial_sides(snapped_instant instant, double opening)
{
    return instant == snapped_instant::start ? sides{0.0, opening, opening} : sides{-opening, 0.0, 0.0};
}

// Where the coast is too short to part the ramp end from the coast end, both accept every phase.
sides trapezoid_sides(const scaling_case &example)
{
    const double a     = example.acceleration_bound;
    const double ramp  = trapezoid_ramp_duration(example.duration, a);
    const double ulp   = std::nextafter(example.duration, std::numeric_limits<double>::infinity()) - example.duration;
    const bool merged  = example.duration - ramp - ramp < 64.0 * ulp;
    const sides joined = sides{a, 0.0, -a};

    switch(example.snapped)
    {
        case snapped_instant::start:
            return sides{0.0, a, a};
        case snapped_instant::ramp_end:
            return merged ? joined : sides{a, 0.0, 0.0};
        case snapped_instant::coast_end:
            return merged ? joined : sides{0.0, -a, -a};
        default:
            return sides{-a, 0.0, 0.0};
    }
}

// Lynch & Park, Modern Robotics, sec. 9.2.2.1: a cubic's second derivative is 6/T^2 at the start and
// -6/T^2 at the finish, and a quintic's is zero at both.
evaluation::case_result compare_cubic(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const scaling_case example = drawn_scaling_case(drawn);
    const sides around         = polynomial_sides(example.snapped, 6.0 / (example.duration * example.duration));

    return compared(time_scalings_of(first).cubic(example.at, example.duration), time_scalings_of(second).cubic(example.at, example.duration), allowed, example, around);
}

evaluation::case_result compare_quintic(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const scaling_case example = drawn_scaling_case(drawn);

    return compared(time_scalings_of(first).quintic(example.at, example.duration), time_scalings_of(second).quintic(example.at, example.duration), allowed, example,
                    polynomial_sides(example.snapped, 0.0));
}

evaluation::case_result compare_trapezoidal(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const scaling_case example = drawn_trapezoid_case(drawn);

    return compared(time_scalings_of(first).trapezoidal(example.at, example.duration, example.speed_bound, example.acceleration_bound),
                    time_scalings_of(second).trapezoidal(example.at, example.duration, example.speed_bound, example.acceleration_bound), allowed, example, trapezoid_sides(example));
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
