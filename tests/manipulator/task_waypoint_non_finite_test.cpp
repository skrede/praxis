#include "praxis/manipulator/evaluation.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/task_trajectory.h"
#include "praxis/manipulator/baseline/task_trajectory.h"

#include "praxis/trajectory/trajectory.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <string_view>

using namespace praxis;
using namespace praxis::evaluation;
using namespace praxis::manipulator;

namespace {

using point     = trajectory::trajectory_sample;
using task_slot = decltype(task_trajectory_ops::task_space_waypoints);

constexpr std::string_view the_row = "trajectory.task_space_waypoints";

// The reference's prepared motion, answering its samples with the first element of one quantity made
// not finite at every time strictly inside the span.
template<trajectory::configuration point::*Member>
class not_finite_motion final : public trajectory::trajectory_generator
{
public:
    explicit not_finite_motion(std::unique_ptr<trajectory::trajectory_generator> held)
            : m_held(std::move(held))
    {
    }

    expected<point, refusal> sample(double t) const override
    {
        expected<point, refusal> read = m_held->sample(t);
        if(read && t > 0.0 && t < m_held->duration())
            ((*read).*Member)[0] = std::numeric_limits<double>::quiet_NaN();

        return read;
    }

    double duration() const override
    {
        return m_held->duration();
    }

private:
    std::unique_ptr<trajectory::trajectory_generator> m_held;
};

template<trajectory::configuration point::*Member>
expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> not_finite_task_space_waypoints(const kinematics &solver, std::span<const transform> waypoints,
                                                                                                     const joint_vector &j0, const joint_limits &limits)
{
    expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> motion = task_space_waypoints(solver, waypoints, j0, limits);
    if(!motion || *motion == nullptr)
        return motion;

    return std::unique_ptr<trajectory::trajectory_generator>(std::make_unique<not_finite_motion<Member>>(std::move(*motion)));
}

struct task_entry
{
    std::string_view name;
    task_slot slot;
};

constexpr std::array task_entries{
        task_entry{"position", &not_finite_task_space_waypoints<&point::position>},
        task_entry{"velocity", &not_finite_task_space_waypoints<&point::velocity>},
        task_entry{"acceleration", &not_finite_task_space_waypoints<&point::acceleration>},
};

constexpr std::array orders{"answering first", "reference first"};

// The task-space waypoint view is the last the aggregate lists.
slot_report task_row(task_slot slot, bool answering_first)
{
    capabilities answering                    = baseline();
    answering.trajectory.task_space_waypoints = slot;
    const capabilities reference              = baseline();
    const auto views                          = answering_first ? evaluation_views(answering, reference) : evaluation_views(reference, answering);

    return evaluate(std::span(views).last(1), default_seed, default_cases_per_slot).slots.at(0);
}

}

TEST_CASE("a_value_that_is_not_finite_in_a_prepared_task_space_motion_is_judged_differing_by_the_task_space_waypoint_row_whichever_side_answers_it")
{
    for(const task_entry &entry : task_entries)
        for(const bool answering_first : {true, false})
        {
            INFO(entry.name << ", " << orders.at(answering_first ? 0 : 1));
            const slot_report row = task_row(entry.slot, answering_first);

            CHECK(row.slot == the_row);
            CHECK(row.verdict == agreement::differed);
            CHECK(row.outcomes.agreed == 0u);
            CHECK(row.outcomes.differed > 0u);
            CHECK(row.outcomes.differed + row.outcomes.both_refused == default_cases_per_slot);
            CHECK(std::isinf(row.worst.magnitude));
        }
}
