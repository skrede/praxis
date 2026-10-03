#include "book_via_points.h"

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/evaluation.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/trajectory/trajectory.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <memory>
#include <vector>
#include <cstddef>
#include <cstdint>

using namespace praxis;
using namespace praxis::evaluation;
using namespace praxis::manipulator;

namespace {

constexpr std::size_t seeds_walked = 40u;

using task_waypoint_slot = expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> (*)(const kinematics &, std::span<const transform>, const joint_vector &,
                                                                                                    const joint_limits &);

// Each pose is resolved from the previous resolution through the solver the slot is handed.
template<std::size_t Form>
expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> literal_task_space_waypoints(const kinematics &solver, std::span<const transform> waypoints, const joint_vector &j0,
                                                                                                  const joint_limits &limits)
{
    std::vector<joint_vector> rows;
    if(waypoints.size() == 1u)
        rows.push_back(j0);

    joint_vector seed = j0;
    for(const transform &pose : waypoints)
    {
        const expected<joint_vector, refusal> reached = manipulator::baseline().motion.task_space_pose(solver, pose, seed);
        if(!reached)
            return unexpected(reached.error());

        seed = *reached;
        rows.push_back(seed);
    }

    return tests::book_via_points<tests::via_forms, Form>(rows, j0, limits);
}

constexpr std::array<task_waypoint_slot, 4> literal_slots{&literal_task_space_waypoints<0>, &literal_task_space_waypoints<1>, &literal_task_space_waypoints<2>,
                                                          &literal_task_space_waypoints<3>};

// The task trajectory view is the last the aggregate lists.
slot_report judged(task_waypoint_slot slot, std::uint64_t seed)
{
    const capabilities reference            = manipulator::baseline();
    capabilities literal                    = manipulator::baseline();
    literal.trajectory.task_space_waypoints = slot;
    const auto views                        = evaluation_views(reference, literal);

    return evaluate(std::span(views).last(1), seed, default_cases_per_slot).slots.at(0);
}

}

TEST_CASE("a_literal_task_space_waypoint_run_in_each_arrangement_is_judged_agreeing_by_the_task_space_waypoint_row_at_each_of_forty_seeds")
{
    for(std::size_t offset = 0; offset < seeds_walked; ++offset)
        for(std::size_t form = 0; form < literal_slots.size(); ++form)
        {
            const slot_report row = judged(literal_slots.at(form), default_seed + offset);

            INFO("seed offset " << offset << ", form " << form);
            CHECK(row.slot == "trajectory.task_space_waypoints");
            CHECK(row.verdict == agreement::agreed);
            CHECK(row.outcomes.differed == 0u);
            CHECK(row.outcomes.agreed + row.outcomes.both_refused == default_cases_per_slot);
        }
}
