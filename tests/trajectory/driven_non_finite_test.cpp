#include "non_finite_generators.h"

#include "praxis/trajectory/evaluation.h"
#include "praxis/trajectory/trajectory.h"
#include "praxis/trajectory/capabilities.h"
#include "praxis/trajectory/pose_trajectory.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <cmath>
#include <cstddef>
#include <string_view>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

using sample     = trajectory::trajectory_sample;
using pose       = trajectory::pose_sample;
using joint_slot = decltype(trajectory::trajectory_ops::joint_space_waypoints);
using pose_slot  = decltype(trajectory::pose_trajectory_ops::decoupled_pose_waypoints);

struct joint_entry
{
    std::string_view name;
    joint_slot slot;
};

constexpr std::array joint_entries{
        joint_entry{"position inside", &fixture::not_finite_joint_space_waypoints<&fixture::made_not_finite<&sample::position, 0>, &fixture::inside>},
        joint_entry{"position at the start", &fixture::not_finite_joint_space_waypoints<&fixture::made_not_finite<&sample::position, 0>, &fixture::at_the_start>},
        joint_entry{"velocity inside", &fixture::not_finite_joint_space_waypoints<&fixture::made_not_finite<&sample::velocity, 0>, &fixture::inside>},
        joint_entry{"velocity at the start", &fixture::not_finite_joint_space_waypoints<&fixture::made_not_finite<&sample::velocity, 0>, &fixture::at_the_start>},
        joint_entry{"acceleration inside", &fixture::not_finite_joint_space_waypoints<&fixture::made_not_finite<&sample::acceleration, 0>, &fixture::inside>},
        joint_entry{"acceleration at the start", &fixture::not_finite_joint_space_waypoints<&fixture::made_not_finite<&sample::acceleration, 0>, &fixture::at_the_start>},
};

struct pose_entry
{
    std::string_view name;
    pose_slot decoupled;
    pose_slot screw;
};

template<void (*Spoil)(pose &)>
constexpr pose_entry pose_entry_of(std::string_view name)
{
    return pose_entry{name, &fixture::not_finite_decoupled_pose_waypoints<Spoil>, &fixture::not_finite_screw_pose_waypoints<Spoil>};
}

constexpr std::array pose_entries{
        pose_entry_of<&fixture::made_not_finite<&pose::position, 0, 0>>("rotation"),
        pose_entry_of<&fixture::made_not_finite<&pose::position, 0, 3>>("translation"),
        pose_entry_of<&fixture::made_not_finite<&pose::velocity, 0>>("angular velocity"),
        pose_entry_of<&fixture::made_not_finite<&pose::velocity, 3>>("linear velocity"),
        pose_entry_of<&fixture::made_not_finite<&pose::acceleration, 0>>("angular acceleration"),
        pose_entry_of<&fixture::made_not_finite<&pose::acceleration, 3>>("linear acceleration"),
};

constexpr std::array orders{"answering first", "reference first"};

// The joint-space trajectory view is the last the aggregate lists.
slot_report joint_row(joint_slot slot, bool answering_first)
{
    trajectory::capabilities answering         = trajectory::baseline();
    answering.trajectory.joint_space_waypoints = slot;
    const trajectory::capabilities reference   = trajectory::baseline();
    const auto views                           = answering_first ? trajectory::evaluation_views(answering, reference) : trajectory::evaluation_views(reference, answering);

    return evaluate(std::span(views).last(1), default_seed, default_cases_per_slot).slots.at(0);
}

// The two pose rows are the pose trajectory view, the third the aggregate lists.
evaluation_report pose_rows(const pose_entry &entry, bool answering_first)
{
    trajectory::capabilities answering                 = trajectory::baseline();
    answering.pose_trajectory.decoupled_pose_waypoints = entry.decoupled;
    answering.pose_trajectory.screw_pose_waypoints     = entry.screw;
    const trajectory::capabilities reference           = trajectory::baseline();
    const auto views                                   = answering_first ? trajectory::evaluation_views(answering, reference) : trajectory::evaluation_views(reference, answering);

    return evaluate(std::span(views).subspan(2, 1), default_seed, default_cases_per_slot);
}

}

TEST_CASE("a_value_that_is_not_finite_in_a_driven_configuration_is_judged_differing_by_the_joint_space_waypoint_row_whichever_side_answers_it")
{
    for(const joint_entry &entry : joint_entries)
        for(const bool answering_first : {true, false})
        {
            INFO(entry.name << ", " << orders.at(answering_first ? 0 : 1));
            const slot_report row = joint_row(entry.slot, answering_first);

            CHECK(row.verdict == agreement::differed);
            CHECK(row.outcomes.agreed == 0u);
            CHECK(row.outcomes.differed == default_cases_per_slot);
            CHECK(std::isinf(row.worst.magnitude));
        }
}

TEST_CASE("a_value_that_is_not_finite_in_a_driven_pose_is_judged_differing_by_both_pose_rows_whichever_side_answers_it")
{
    constexpr std::array rows{"pose_trajectory.decoupled_pose_waypoints", "pose_trajectory.screw_pose_waypoints"};

    for(const pose_entry &entry : pose_entries)
        for(const bool answering_first : {true, false})
        {
            const evaluation_report reported = pose_rows(entry, answering_first);

            REQUIRE(reported.slots.size() == rows.size());
            for(std::size_t index = 0; index < rows.size(); ++index)
            {
                const slot_report &row = reported.slots.at(index);
                INFO(entry.name << ", " << orders.at(answering_first ? 0 : 1) << ", " << rows.at(index));

                CHECK(row.slot == rows.at(index));
                CHECK(row.verdict == agreement::differed);
                CHECK(row.outcomes.agreed == 0u);
                CHECK(row.outcomes.differed == default_cases_per_slot);
                CHECK(std::isinf(row.worst.magnitude));
                CHECK(std::isinf(row.worst.linear_error_metres));
            }
        }
}
