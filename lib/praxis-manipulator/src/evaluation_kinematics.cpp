#include "evaluation_cases.h"
#include "evaluation_tables.h"

#include "praxis/manipulator/slots.h"
#include "praxis/manipulator/baseline/kinematics.h"

#include "praxis/evaluation/comparators.h"

#include "praxis/rigid_motion/capabilities.h"

#include <span>
#include <array>
#include <vector>
#include <cstddef>

namespace praxis::manipulator {

namespace {

// One screw implementation and one frame implementation serve both sides of every row that reaches
// them, so a row measures its own slot rather than a capability neither side is under test for.
const rigid_motion::screw_ops &shared_screw()
{
    static const rigid_motion::screw_ops screw = rigid_motion::baseline().screw;

    return screw;
}

const rigid_motion::frame_ops &shared_frames()
{
    static const rigid_motion::frame_ops frames = rigid_motion::baseline().frame;

    return frames;
}

const forward_kinematics_ops &forward_kinematics_of(const void *value)
{
    return *static_cast<const forward_kinematics_ops *>(value);
}

const differential_kinematics_ops &differential_kinematics_of(const void *value)
{
    return *static_cast<const differential_kinematics_ops *>(value);
}

// The rows are in the enumerator order of forward_kinematics_slot, and each name is spelled exactly
// as the descriptor table spells it. Every slot this aggregate describes is compared here, which is
// what the assertion below the table holds. The row asked over body screws derives them once and
// hands the same derivation to both sides, so it measures its own slot rather than that derivation.
constexpr std::array forward_kinematics_table{
        evaluation::slot_evaluation{"fk.forward_kinematics", evaluation::residual_kind::pose, evaluation::tolerance_of(evaluation::residual_kind::pose),
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const evaluation_case example = drawn_case(drawn);
                                        const expected<transform, refusal> held =
                                                forward_kinematics_of(first).forward_kinematics(example.chain.home, example.chain.space_screws, example.joints);
                                        const expected<transform, refusal> against =
                                                forward_kinematics_of(second).forward_kinematics(example.chain.home, example.chain.space_screws, example.joints);

                                        return evaluation::agreed_or_refused(held, against, evaluation::pose_residual, allowed);
                                    }},
        evaluation::slot_evaluation{
                "fk.body_forward_kinematics", evaluation::residual_kind::pose, evaluation::tolerance_of(evaluation::residual_kind::pose),
                [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                {
                    const evaluation_case example                         = drawn_case(drawn);
                    const expected<std::vector<screw_axis>, refusal> body = body_screws_from_space(shared_screw(), shared_frames(), example.chain.home, example.chain.space_screws);
                    if(!body)
                        return unusable(evaluation::residual_kind::pose);

                    const expected<transform, refusal> held    = forward_kinematics_of(first).body_forward_kinematics(shared_frames(), example.chain.home, *body, example.joints);
                    const expected<transform, refusal> against = forward_kinematics_of(second).body_forward_kinematics(shared_frames(), example.chain.home, *body, example.joints);

                    return evaluation::agreed_or_refused(held, against, evaluation::pose_residual, allowed);
                }},
        evaluation::slot_evaluation{"fk.body_screws_from_space", evaluation::residual_kind::element_wise, evaluation::tolerance_of(evaluation::residual_kind::element_wise),
                                    &compare_body_screws_from_space},
};

static_assert(forward_kinematics_table.size() == static_cast<std::size_t>(forward_kinematics_slot::count));

constexpr evaluation::capability_evaluations<forward_kinematics_ops> evaluated_forward_kinematics{"manipulator", forward_kinematics_table};

// The rows are in the enumerator order of differential_kinematics_slot, and each name is spelled
// exactly as the descriptor table spells it. Every slot this aggregate describes is compared here,
// which is what the assertion below the table holds. The row asked over body screws derives them
// once and hands the same derivation to both sides, so it measures its own slot rather than that
// derivation.
constexpr std::array differential_kinematics_table{
        evaluation::slot_evaluation{"dk.space_jacobian", evaluation::residual_kind::element_wise,
                                    evaluation::tolerance_pair{accumulated_element_wise_tolerance, accumulated_element_wise_tolerance},
                                    [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                                    {
                                        const evaluation_case example             = drawn_case(drawn);
                                        const expected<jacobian, refusal> held    = differential_kinematics_of(first).space_jacobian(example.chain.space_screws, example.joints);
                                        const expected<jacobian, refusal> against = differential_kinematics_of(second).space_jacobian(example.chain.space_screws, example.joints);

                                        return evaluation::agreed_or_refused(held, against, evaluation::element_wise_residual, allowed);
                                    }},
        evaluation::slot_evaluation{
                "dk.body_jacobian", evaluation::residual_kind::element_wise, evaluation::tolerance_pair{accumulated_element_wise_tolerance, accumulated_element_wise_tolerance},
                [](const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed) -> evaluation::case_result
                {
                    const evaluation_case example                         = drawn_case(drawn);
                    const expected<std::vector<screw_axis>, refusal> body = body_screws_from_space(shared_screw(), shared_frames(), example.chain.home, example.chain.space_screws);
                    if(!body)
                        return unusable(evaluation::residual_kind::element_wise);

                    const expected<jacobian, refusal> held    = differential_kinematics_of(first).body_jacobian(*body, example.joints);
                    const expected<jacobian, refusal> against = differential_kinematics_of(second).body_jacobian(*body, example.joints);

                    return evaluation::agreed_or_refused(held, against, evaluation::element_wise_residual, allowed);
                }},
};

static_assert(differential_kinematics_table.size() == static_cast<std::size_t>(differential_kinematics_slot::count));

constexpr evaluation::capability_evaluations<differential_kinematics_ops> evaluated_differential_kinematics{"manipulator", differential_kinematics_table};

// The rows are in the enumerator order of inverse_kinematics_slot, and each name is spelled exactly
// as the descriptor table spells it. The table compares no more slots than the aggregate describes,
// which is what the assertion below it holds, and every described slot no row here compares is named
// by `unevaluated_slots`.
constexpr std::array inverse_kinematics_table{
        evaluation::slot_evaluation{"ik.inverse_kinematics", evaluation::residual_kind::pose, evaluation::tolerance_pair{solved_pose_tolerance_radians, solved_pose_tolerance_metres},
                                    &compare_inverse_kinematics},
};

static_assert(inverse_kinematics_table.size() <= static_cast<std::size_t>(inverse_kinematics_slot::count));

constexpr evaluation::capability_evaluations<inverse_kinematics_ops> evaluated_inverse_kinematics{"manipulator", inverse_kinematics_table};

}

const evaluation::capability_evaluations<forward_kinematics_ops> &forward_kinematics_evaluations()
{
    return evaluated_forward_kinematics;
}

const evaluation::capability_evaluations<differential_kinematics_ops> &differential_kinematics_evaluations()
{
    return evaluated_differential_kinematics;
}

const evaluation::capability_evaluations<inverse_kinematics_ops> &inverse_kinematics_evaluations()
{
    return evaluated_inverse_kinematics;
}

}
