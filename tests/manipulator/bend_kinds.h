#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_BEND_KINDS_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_BEND_KINDS_H

#include <array>
#include <cstddef>
#include <algorithm>
#include <string_view>

// One displacement per quantity a manipulator row's residual measures, and the rows each of them
// moves. A displacement is set through the array rather than compiled in because a slot is a plain
// function pointer and carries nothing of its own, so a run choosing which rows to move sets the
// entries it wants and leaves the rest at zero.
namespace praxis::fixture {

enum class bent : std::size_t
{
    element_wise,
    geodesic,
    pose_radians,
    pose_metres,
    configuration,
    prepared_motion,
    count,
};

inline std::array<double, static_cast<std::size_t>(bent::count)> bend_by{};

inline double bent_by(bent which)
{
    return bend_by[static_cast<std::size_t>(which)];
}

inline void bend_every_row_by(const std::array<double, static_cast<std::size_t>(bent::count)> &by)
{
    std::copy(by.begin(), by.end(), bend_by.begin());
}

// The rows one bend moves, which is what a ledger over a bent aggregate must name and nothing else.
// The two halves of a pose discrepancy are carried by the same rows and separated by which entry of
// the array stands above zero.
inline bool is_bent(bent which, std::string_view slot)
{
    switch(which)
    {
        case bent::element_wise:
            return slot == "dk.space_jacobian" || slot == "dk.body_jacobian" || slot == "fk.body_screws_from_space" || slot == "robot.position_from_pose";
        case bent::geodesic:
            return slot == "robot.orientation_from_pose";
        case bent::pose_radians:
        case bent::pose_metres:
            return slot == "fk.forward_kinematics" || slot == "fk.body_forward_kinematics" || slot == "robot.tool_pose_from_flange_pose" || slot == "robot.flange_pose_from_tool_pose" ||
                    slot == "modeling.build_chain";
        case bent::prepared_motion:
            return slot == "trajectory.task_space_waypoints";
        case bent::configuration:
            return slot == "ik.inverse_kinematics" || slot == "robot.ik_solve_pose" || slot == "robot.ik_solve_flange_pose" || slot == "motion.task_space_pose" ||
                    slot == "motion.task_space_screw" || slot == "motion.tool_frame_displace";
        case bent::count:
            break;
    }

    return false;
}

}

#endif
