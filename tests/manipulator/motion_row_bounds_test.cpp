#include "evaluation_cases.h"
#include "evaluation_tables.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/motion.h"

#include "praxis/evaluation/generation.h"
#include "praxis/evaluation/slot_evaluation.h"

#include "praxis/rigid_motion/baseline/frame.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <cstddef>
#include <cstdint>

using namespace praxis;
using namespace praxis::manipulator;
using namespace praxis::evaluation;

namespace {

constexpr std::uint64_t recorded_seed = 0xC0FFEEu;
constexpr std::size_t cases_per_row   = 24u;

// How far off the pose it was asked for the bindings below stand. The displacement post-multiplies
// the target, so a turn leaves the origin where it was and a travel leaves the orientation.
double off_target_radians = 0.0;
double off_target_metres  = 0.0;

transform standing_off(const transform &asked_for)
{
    const rotation turned(Eigen::AngleAxisd(off_target_radians, Eigen::Vector3d::UnitX()).toRotationMatrix());

    return transform(asked_for * rigid_motion::transformation_matrix_from_rotation_position(turned, off_target_metres * Eigen::Vector3d::UnitX()));
}

expected<joint_vector, refusal> pose_standing_off(const kinematics &solver, const transform &pose, const joint_vector &j0)
{
    return task_space_pose(solver, standing_off(pose), j0);
}

// The target is formed as the reference forms it and then displaced, so what the row answers stands
// off the pose the comparison holds it against by the displacement and by nothing else.
expected<joint_vector, refusal> screw_standing_off(const rigid_motion::screw_ops &screw, const kinematics &solver, const transform &start_pose, const Eigen::Vector3d &w,
                                                   const Eigen::Vector3d &q, double theta_radians, double h, const joint_vector &j0)
{
    const expected<screw_axis, refusal> axis = screw.screw_axis_from_point_direction_pitch(q, w, h);
    if(!axis)
        return unexpected(axis.error());

    return task_space_pose(solver, standing_off(screw.matrix_exponential_screw(*axis, theta_radians) * start_pose), j0);
}

expected<joint_vector, refusal> displace_standing_off(const kinematics &solver, const transform &start_pose, const Eigen::Vector3d &offset, const rotation &orientation,
                                                      const joint_vector &j0)
{
    return task_space_pose(solver, standing_off(start_pose * rigid_motion::transformation_matrix_from_rotation_position(orientation, offset)), j0);
}

constexpr motion_ops standing_off_target{
        .task_space_pose     = &pose_standing_off,
        .task_space_screw    = &screw_standing_off,
        .tool_frame_displace = &displace_standing_off,
};

std::vector<agreement> over_the_run(const slot_evaluation &row, const motion_ops &first, const motion_ops &second)
{
    std::vector<agreement> seen;
    seen.reserve(cases_per_row);

    for(std::size_t index = 0; index < cases_per_row; ++index)
    {
        case_source drawn = case_source::at_case(recorded_seed, row.name, spread::bulk, index);

        seen.push_back(row.compare(&first, &second, drawn, row.allowed).verdict);
    }

    return seen;
}

std::size_t how_many(const std::vector<agreement> &seen, agreement which)
{
    std::size_t counted = 0;

    for(const agreement verdict : seen)
        counted += verdict == which ? 1u : 0u;

    return counted;
}

}

// The displacement standing at the bound and the one a decade beneath it, written out rather than
// derived from the bounds, so a bound moved up past the first or down onto the second fails here.
// A binding standing at the bound off the pose is reported on some of the run because the solve's
// own convergence residual stands beside the displacement and the two together cross; one a decade
// beneath is reported on none of it, because the two together stay under. A row folds to a
// difference on one case differing, so reporting some of the run is what the row answers.
constexpr double at_the_bound      = 1.0e-6;
constexpr double beneath_the_bound = 1.0e-7;

// Each row is asked for a pose and answers a configuration, so the displacement that decides the
// verdict is the one between the pose the answer reaches and the pose the row was asked for.
TEST_CASE("each_motion_row_reports_a_pose_standing_at_the_bound_it_carries_and_leaves_one_a_decade_beneath")
{
    const motion_ops reference = baseline().motion;

    for(const bool angular : {true, false})
        for(const bool above : {true, false})
        {
            const double displaced = above ? at_the_bound : beneath_the_bound;
            off_target_radians     = angular ? displaced : 0.0;
            off_target_metres      = angular ? 0.0 : displaced;

            for(const slot_evaluation &row : motion_evaluations().slots)
            {
                INFO("row " << row.name << ", angular " << angular << ", above " << above);
                const std::vector<agreement> seen = over_the_run(row, reference, standing_off_target);

                REQUIRE(how_many(seen, agreement::agreed) > 0u);
                if(above)
                    REQUIRE(how_many(seen, agreement::differed) > 0u);
                else
                    REQUIRE(how_many(seen, agreement::differed) == 0u);
            }
        }

    off_target_radians = 0.0;
    off_target_metres  = 0.0;
}

// The bound governs only the cases both sides answered. On the row asked for a pose from a drawn
// seed the two sides disagree about reachability often enough that the share has to be read rather
// than assumed, and on the two asked for a short displacement from a reached pose it is far smaller.
TEST_CASE("every_motion_row_answers_from_both_sides_over_the_run_the_bound_is_read_at")
{
    const motion_ops reference = baseline().motion;

    for(const slot_evaluation &row : motion_evaluations().slots)
    {
        INFO("row " << row.name);
        const std::vector<agreement> seen = over_the_run(row, reference, reference);

        REQUIRE(how_many(seen, agreement::agreed) > 0u);
        REQUIRE(how_many(seen, agreement::differed) == 0u);
        REQUIRE(how_many(seen, agreement::unusable) == 0u);
    }
}
