#include "bent_manipulator.h"

#include "praxis/manipulator/evaluation.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/task_trajectory.h"
#include "praxis/manipulator/baseline/task_trajectory.h"

#include "praxis/trajectory/trajectory.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <cmath>
#include <memory>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <string_view>

using namespace praxis;
using namespace praxis::evaluation;
using namespace praxis::manipulator;

namespace {

constexpr std::uint64_t recorded_seed = 0xC0FFEEu;
constexpr std::size_t cases_per_row   = 16u;
constexpr std::string_view the_row    = "trajectory.task_space_waypoints";

// A prepared motion answering the reference's over a span twice as long, which is the one thing the
// comparison reads before it samples either side.
class stretched_motion final : public trajectory::trajectory_generator
{
public:
    explicit stretched_motion(std::unique_ptr<trajectory::trajectory_generator> held)
            : m_held(std::move(held))
    {
    }

    expected<trajectory::trajectory_sample, refusal> sample(double t) const override
    {
        return m_held->sample(t);
    }

    double duration() const override
    {
        return 2.0 * m_held->duration();
    }

private:
    std::unique_ptr<trajectory::trajectory_generator> m_held;
};

expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> stretched_task_space_waypoints(const kinematics &solver, std::span<const transform> waypoints,
                                                                                                    const joint_vector &j0, const joint_limits &limits)
{
    expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> motion = task_space_waypoints(solver, waypoints, j0, limits);
    if(!motion || *motion == nullptr)
        return motion;

    return std::unique_ptr<trajectory::trajectory_generator>(std::make_unique<stretched_motion>(std::move(*motion)));
}

// A factory answering no motion at all. Nothing may read through the pointer it names.
expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> no_motion_at_all(const kinematics &, std::span<const transform>, const joint_vector &, const joint_limits &)
{
    return std::unique_ptr<trajectory::trajectory_generator>();
}

slot_report row_of(const capabilities &reference, const capabilities &other)
{
    const evaluation_report reported = evaluate(evaluation_views(reference, other), recorded_seed, cases_per_row);

    for(const slot_report &slot : reported.slots)
        if(slot.slot == the_row)
            return slot;

    return slot_report{};
}

capabilities factories_bound_to(expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> (*factory)(const kinematics &, std::span<const transform>, const joint_vector &,
                                                                                                                const joint_limits &))
{
    capabilities arm = baseline();

    arm.trajectory.task_space_waypoints = factory;

    return arm;
}

}

TEST_CASE("the_reference_answers_the_via_point_row_over_the_waypoint_sets_the_run_draws")
{
    fixture::bend_every_row_by({});

    const capabilities reference = baseline();
    const slot_report row        = row_of(reference, reference);

    REQUIRE(row.slot == the_row);
    REQUIRE(row.verdict == agreement::agreed);
    REQUIRE(row.outcomes.agreed > 0u);
    REQUIRE(row.outcomes.unusable == 0u);
}

// The refusal policy runs before either pointer is read, so a pair only one side of which answered
// is an outcome of its own and never a motion driven against nothing.
TEST_CASE("a_generator_pair_where_exactly_one_side_answered_is_one_refused")
{
    fixture::bend_every_row_by({});

    const slot_report row = row_of(baseline(), factories_bound_to(&inert::task_space_waypoints));

    REQUIRE(row.slot == the_row);
    REQUIRE(row.outcomes.one_refused > 0u);
    REQUIRE(row.outcomes.differed == 0u);
    REQUIRE(row.outcomes.agreed == 0u);
}

// A factory that answered a null pointer named no motion, so the pair is a difference no residual
// measures rather than a pointer read on the strength of nobody's promise.
TEST_CASE("a_side_answering_no_motion_at_all_is_a_difference_and_not_a_read")
{
    fixture::bend_every_row_by({});

    const slot_report row = row_of(baseline(), factories_bound_to(&no_motion_at_all));

    REQUIRE(row.slot == the_row);
    REQUIRE(row.verdict == agreement::differed);
    REQUIRE(std::isinf(row.worst.magnitude));
}

// The durations are read before any time is sampled: two spans that differ share no interval to
// compare over, so the row reports a magnitude beyond every bound rather than a number.
TEST_CASE("two_generators_whose_durations_differ_are_reported_with_an_unbounded_magnitude")
{
    fixture::bend_every_row_by({});

    const slot_report row = row_of(baseline(), factories_bound_to(&stretched_task_space_waypoints));

    REQUIRE(row.slot == the_row);
    REQUIRE(row.verdict == agreement::differed);
    REQUIRE(std::isinf(row.worst.magnitude));
}
