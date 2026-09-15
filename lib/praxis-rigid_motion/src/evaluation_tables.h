#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_EVALUATION_TABLES_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_EVALUATION_TABLES_H

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/screw.h"

#include "praxis/evaluation/slot_evaluation.h"

namespace praxis::rigid_motion {

// The bound the adjoint map is judged at, in the dimensionless unit an element-wise difference carries:
// its answer is a product of a six-by-six adjoint with a twist, one multiplication past the adjoint itself.
inline constexpr double adjoint_map_element_wise_tolerance = 1.0e-12;

const evaluation::capability_evaluations<frame_ops> &frame_evaluations();
const evaluation::capability_evaluations<screw_ops> &screw_evaluations();

inline evaluation::case_result judged(const evaluation::residual &difference, const evaluation::tolerance_pair &allowed)
{
    return evaluation::case_result{evaluation::verdict_of(difference, allowed), difference};
}

evaluation::case_result compare_matrix_exponential_so3(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed);
evaluation::case_result compare_matrix_exponential_se3(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed);
evaluation::case_result compare_matrix_exponential_screw(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed);
evaluation::case_result compare_matrix_logarithm_so3(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed);
evaluation::case_result compare_matrix_logarithm_se3_rp(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed);
evaluation::case_result compare_matrix_logarithm_se3(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed);

}

#endif
