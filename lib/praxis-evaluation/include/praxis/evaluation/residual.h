#ifndef HPP_GUARD_PRAXIS_EVALUATION_RESIDUAL_H
#define HPP_GUARD_PRAXIS_EVALUATION_RESIDUAL_H

#include <cstdint>

namespace praxis::evaluation {

enum class residual_kind : std::uint8_t
{
    element_wise,
    geodesic,
    pose,
    axis_up_to_sign,
    log_up_to_branch,
};

// The unit of `magnitude` is fixed by the kind: the greatest absolute element difference for
// `element_wise`, and a rotation measured in radians for `geodesic`, `axis_up_to_sign` and
// `log_up_to_branch`. A `pose` discrepancy takes two numbers -- `magnitude` is its rotational half
// in radians and `linear_error_metres` its translational half in metres. A `log_up_to_branch`
// discrepancy carries the second field too, since a logarithm may name a pose; its rotation form
// leaves that half at zero. No other kind reads it.
struct residual
{
    residual_kind kind;
    double magnitude;
    double linear_error_metres;
};

// `unusable` is the harness's own outcome and not either side's: the case's shared input could not
// be built, so neither side was asked about it and neither is answerable for it. `beyond_measurement`
// is the harness's own as well and means something different: the input was built and both sides
// answered, and the run drew more cases than the row's bound was measured over, so no verdict read
// at that bound is claimed for it.
enum class agreement : std::uint8_t
{
    not_exercised,
    agreed,
    differed,
    both_refused,
    refused_differently,
    one_refused,
    unusable,
    beyond_measurement,
};

// What a residual of each half may be and still count as agreement, in the units `residual` states.
struct tolerance_pair
{
    double magnitude;
    double linear_metres;
};

// Each bound is in the unit its own name spells. The two that spell neither are dimensionless: the
// element-wise bound is an absolute difference between two numbers, and the axis bound is the norm
// of a six-vector whose two halves carry different units and is therefore read as a number rather
// than as a length.
inline constexpr double element_wise_tolerance             = 1.0e-13;
inline constexpr double geodesic_tolerance_radians         = 1.0e-13;
inline constexpr double pose_tolerance_radians             = 1.0e-13;
inline constexpr double pose_tolerance_metres              = 1.0e-12;
inline constexpr double axis_up_to_sign_tolerance          = 1.0e-12;
inline constexpr double log_up_to_branch_tolerance_radians = 1.0e-13;
inline constexpr double log_up_to_branch_tolerance_metres  = 1.0e-12;

// A kind carrying no translational half repeats its own bound in the second field, which nothing
// reads: `verdict_of` judges that half for `pose` and `log_up_to_branch` alone.
constexpr tolerance_pair tolerance_of(residual_kind kind)
{
    switch(kind)
    {
        case residual_kind::geodesic:
            return tolerance_pair{geodesic_tolerance_radians, geodesic_tolerance_radians};
        case residual_kind::pose:
            return tolerance_pair{pose_tolerance_radians, pose_tolerance_metres};
        case residual_kind::axis_up_to_sign:
            return tolerance_pair{axis_up_to_sign_tolerance, axis_up_to_sign_tolerance};
        case residual_kind::log_up_to_branch:
            return tolerance_pair{log_up_to_branch_tolerance_radians, log_up_to_branch_tolerance_metres};
        case residual_kind::element_wise:
            break;
    }

    return tolerance_pair{element_wise_tolerance, element_wise_tolerance};
}

// The comparison is inclusive at the tolerance: a residual exactly equal to what is allowed agrees,
// and one a single representable step above it differs. A `pose` residual and a `log_up_to_branch`
// residual must satisfy both halves; every other kind is judged on `magnitude` alone.
agreement verdict_of(const residual &seen, const tolerance_pair &allowed);

}

#endif
