#include "praxis/trajectory/baseline/trajectory.h"

#include "praxis/trajectory/evaluation.h"
#include "praxis/trajectory/trajectory.h"
#include "praxis/trajectory/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <algorithm>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

double moved_by = 0.0;

using answering = trajectory::trajectory_sample (*)(trajectory::trajectory_sample point, double t, double duration);

trajectory::trajectory_sample displaced(trajectory::trajectory_sample point, double, double)
{
    point.acceleration[0] += moved_by * std::max(1.0, std::abs(point.acceleration[0]));

    return point;
}

trajectory::trajectory_sample not_finite_inside(trajectory::trajectory_sample point, double t, double duration)
{
    if(t > 0.0 && t < duration)
        point.acceleration[0] = std::numeric_limits<double>::quiet_NaN();

    return point;
}

template<answering Answer>
class answering_generator : public trajectory::trajectory_generator
{
public:
    explicit answering_generator(std::unique_ptr<trajectory::trajectory_generator> held)
            : m_held(std::move(held))
    {
    }

    expected<trajectory::trajectory_sample, refusal> sample(double t) const override
    {
        const expected<trajectory::trajectory_sample, refusal> read = m_held->sample(t);
        if(!read)
            return read;

        return Answer(*read, t, m_held->duration());
    }

    double duration() const override
    {
        return m_held->duration();
    }

private:
    std::unique_ptr<trajectory::trajectory_generator> m_held;
};

template<answering Answer>
expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> answered(std::span<const trajectory::configuration> waypoints, const trajectory::configuration &j0,
                                                                              const trajectory::configuration_limits &limits)
{
    expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> held = trajectory::joint_space_waypoints(waypoints, j0, limits);
    if(!held)
        return held;

    return std::unique_ptr<trajectory::trajectory_generator>(std::make_unique<answering_generator<Answer>>(std::move(*held)));
}

// Only the joint-space trajectory view is evaluated: it is the last the aggregate lists.
template<answering Answer>
slot_report judged()
{
    trajectory::capabilities literal         = trajectory::baseline();
    literal.trajectory.joint_space_waypoints = &answered<Answer>;
    const trajectory::capabilities reference = trajectory::baseline();
    const auto views                         = trajectory::evaluation_views(literal, reference);

    return evaluate(std::span(views).last(1), default_seed, default_cases_per_slot).slots.at(0);
}

slot_report judged_displaced_by(double fraction)
{
    moved_by              = fraction;
    const slot_report row = judged<&displaced>();
    moved_by              = 0.0;

    return row;
}

}

TEST_CASE("an_acceleration_moved_by_a_fraction_of_its_own_size_a_decade_above_the_joint_space_waypoint_bound_is_judged_differing_on_every_case")
{
    const slot_report row = judged_displaced_by(1.0e-3);

    REQUIRE(row.slot == "trajectory.joint_space_waypoints");
    REQUIRE(row.outcomes.differed == default_cases_per_slot);
}

TEST_CASE("an_acceleration_moved_by_a_fraction_of_its_own_size_a_decade_beneath_the_joint_space_waypoint_bound_is_judged_agreeing_on_every_case")
{
    const slot_report row = judged_displaced_by(1.0e-5);

    REQUIRE(row.slot == "trajectory.joint_space_waypoints");
    REQUIRE(row.verdict == agreement::agreed);
    REQUIRE(row.outcomes.agreed == default_cases_per_slot);
}

TEST_CASE("an_acceleration_that_is_not_finite_at_interior_samples_only_is_judged_differing_on_every_case_by_the_joint_space_waypoint_row")
{
    const slot_report row = judged<&not_finite_inside>();

    REQUIRE(row.slot == "trajectory.joint_space_waypoints");
    REQUIRE(row.outcomes.differed == default_cases_per_slot);
}
