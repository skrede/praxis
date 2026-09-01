#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_EVALUATION_TABLES_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_EVALUATION_TABLES_H

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/screw.h"

#include "praxis/evaluation/slot_evaluation.h"

namespace praxis::rigid_motion {

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
