#include "praxis/manipulator.h"

#include "praxis/trajectory/slots.h"

#include "praxis/rigid_motion/slots.h"

#include <catch2/catch_test_macros.hpp>

#include <set>
#include <span>
#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <string_view>

using namespace praxis;
using namespace praxis::manipulator;
using namespace praxis::rigid_motion;

namespace {

matrix3 constant_skew(const Eigen::Vector3d &)
{
    return matrix3::Constant(4.0);
}

expected<joint_vector, refusal> zeroed_pose(const kinematics &, const transform &, const joint_vector &j0)
{
    return joint_vector(joint_vector::Zero(j0.size()));
}

// The five capabilities this file composes, drawn from all three extensions and held where the views
// into them outlive every report assembled from them.
const motion_ops unbound_motion{};
const frame_ops unbound_frames{};
const screw_ops unbound_screws{};
const trajectory::path_ops unbound_shapes{};
const trajectory::pose_trajectory_ops unbound_poses{};

template<slot_enumeration Slot>
constexpr std::size_t slots_in = static_cast<std::size_t>(Slot::count);

constexpr std::size_t rigid_motion_share = slots_in<frame_slot> + slots_in<screw_slot>;
constexpr std::size_t manipulator_share  = slots_in<motion_slot>;
constexpr std::size_t trajectory_share   = slots_in<trajectory::path_slot> + slots_in<trajectory::pose_trajectory_slot>;
constexpr std::size_t composed_share     = rigid_motion_share + manipulator_share + trajectory_share;

std::string qualified(const defaulted_slot &entry)
{
    return std::string(entry.extension).append(".").append(entry.slot);
}

std::set<std::string> qualified_names(std::span<const capability_view> views)
{
    std::set<std::string> names;
    for(const defaulted_slot &entry : defaulted_slots(views))
    {
        names.insert(qualified(entry));
    }
    return names;
}

std::size_t entries_labeled(std::span<const capability_view> views, std::string_view label)
{
    std::size_t counted = 0;
    for(const defaulted_slot &entry : defaulted_slots(views))
    {
        counted += entry.extension == label ? 1u : 0u;
    }
    return counted;
}

std::array<capability_view, 5> composed()
{
    return {view_of(unbound_motion), view_of(unbound_shapes), view_of(unbound_frames), view_of(unbound_screws), view_of(unbound_poses)};
}

}

TEST_CASE("one_report_spans_three_extensions_and_counts_every_slot_the_caller_composed")
{
    const std::array<capability_view, 5> reported = composed();
    std::size_t described                         = 0;

    for(const capability_view &view : reported)
    {
        REQUIRE_FALSE(view.extension().empty());
        REQUIRE_FALSE(view.slots().empty());
        described += view.slots().size();
    }
    REQUIRE(described == composed_share);
    REQUIRE(count_defaults(reported) == composed_share);
    REQUIRE(defaulted_slots(reported).size() == composed_share);
    REQUIRE(entries_labeled(reported, "rigid_motion") == rigid_motion_share);
    REQUIRE(entries_labeled(reported, "manipulator") == manipulator_share);
    REQUIRE(entries_labeled(reported, "trajectory") == trajectory_share);
}

TEST_CASE("every_entry_carries_a_label_and_a_name_and_the_pair_is_unique_across_the_report")
{
    const std::array<capability_view, 5> reported = composed();
    const std::vector<defaulted_slot> report      = defaulted_slots(reported);
    std::set<std::string> pairs;

    REQUIRE_FALSE(report.empty());
    for(const defaulted_slot &entry : report)
    {
        REQUIRE_FALSE(entry.extension.empty());
        REQUIRE_FALSE(entry.slot.empty());
        REQUIRE(pairs.insert(qualified(entry)).second);
    }
    REQUIRE(pairs.size() == report.size());
}

TEST_CASE("the_report_reaches_a_slot_of_each_extension_through_its_label_rather_than_its_slot_name")
{
    const std::array<capability_view, 5> reported = composed();
    const std::set<std::string> named             = qualified_names(reported);
    std::set<std::string_view> labels;

    for(const defaulted_slot &entry : defaulted_slots(reported))
    {
        labels.insert(entry.extension);
    }
    REQUIRE(labels.size() == 3u);
    REQUIRE(named.count("rigid_motion.frame.rotate_z") == 1u);
    REQUIRE(named.count("rigid_motion.screw.adjoint_map") == 1u);
    REQUIRE(named.count("manipulator.motion.task_space_screw") == 1u);
    REQUIRE(named.count("trajectory.path.decoupled") == 1u);
    REQUIRE(named.count("trajectory.pose_trajectory.decoupled_pose_waypoints") == 1u);
}

TEST_CASE("overriding_one_slot_of_one_extension_leaves_the_others_reported_whole")
{
    const screw_ops screws{.skew_symmetric = &constant_skew};
    const std::array<capability_view, 5> reported{view_of(unbound_motion), view_of(unbound_shapes), view_of(unbound_frames), view_of(screws), view_of(unbound_poses)};
    const std::set<std::string> named = qualified_names(reported);

    REQUIRE(count_defaults(reported) == composed_share - 1u);
    REQUIRE(named.count("rigid_motion.screw.skew_symmetric") == 0u);
    REQUIRE(named.count("rigid_motion.screw.adjoint_map") == 1u);
    REQUIRE(entries_labeled(reported, "rigid_motion") == rigid_motion_share - 1u);
    REQUIRE(entries_labeled(reported, "manipulator") == manipulator_share);
    REQUIRE(entries_labeled(reported, "trajectory") == trajectory_share);
}

TEST_CASE("overriding_one_slot_of_another_extension_has_the_symmetric_effect")
{
    const motion_ops motion{.task_space_pose = &zeroed_pose};
    const std::array<capability_view, 5> reported{view_of(motion), view_of(unbound_shapes), view_of(unbound_frames), view_of(unbound_screws), view_of(unbound_poses)};
    const std::set<std::string> named = qualified_names(reported);

    REQUIRE(count_defaults(reported) == composed_share - 1u);
    REQUIRE(named.count("manipulator.motion.task_space_pose") == 0u);
    REQUIRE(named.count("manipulator.motion.task_space_screw") == 1u);
    REQUIRE(entries_labeled(reported, "manipulator") == manipulator_share - 1u);
    REQUIRE(entries_labeled(reported, "rigid_motion") == rigid_motion_share);
    REQUIRE(entries_labeled(reported, "trajectory") == trajectory_share);
}

TEST_CASE("a_report_over_no_views_names_nothing_and_counts_zero")
{
    const std::span<const capability_view> none;

    REQUIRE(count_defaults(none) == 0u);
    REQUIRE(defaulted_slots(none).empty());
}
