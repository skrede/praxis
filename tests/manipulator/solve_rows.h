#ifndef HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_SOLVE_ROWS_H
#define HPP_GUARD_PRAXIS_TESTS_MANIPULATOR_SOLVE_ROWS_H

#include "evaluation_tables.h"

#include "praxis/manipulator/robot.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/generation.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <span>
#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <stdexcept>
#include <string_view>

// The three shipped rows that run a solve, reached through the tables that ship them so a run is
// judged at the bound the table carries rather than at one spelled again here.
namespace praxis::fixture {

using namespace manipulator;

inline constexpr std::uint64_t recorded_seed = 0xC0FFEEu;
inline constexpr std::size_t cases_per_row   = 24u;

inline const evaluation::slot_evaluation &named(std::span<const evaluation::slot_evaluation> table, std::string_view name)
{
    const auto found = std::find_if(table.begin(), table.end(), [name](const evaluation::slot_evaluation &slot) { return slot.name == name; });
    if(found == table.end())
        throw std::logic_error("no shipped row is named " + std::string(name));

    return *found;
}

inline evaluation::case_result one_case(const evaluation::slot_evaluation &row, const void *first, const void *second, std::size_t index)
{
    evaluation::case_source drawn = evaluation::case_source::at_case(recorded_seed, row.name, evaluation::spread::bulk, index);

    return row.compare(first, second, drawn, row.allowed);
}

// The outcome every case of one run reached, in the order the run took them.
inline std::vector<evaluation::agreement> over_the_run(const evaluation::slot_evaluation &row, const void *first, const void *second)
{
    std::vector<evaluation::agreement> seen;
    seen.reserve(cases_per_row);
    for(std::size_t index = 0; index < cases_per_row; ++index)
        seen.push_back(one_case(row, first, second, index).verdict);

    return seen;
}

inline std::size_t how_many(std::span<const evaluation::agreement> seen, evaluation::agreement reached)
{
    return static_cast<std::size_t>(std::count(seen.begin(), seen.end(), reached));
}

inline inverse_kinematics_ops chain_bound_to(decltype(inverse_kinematics_ops::inverse_kinematics) solving)
{
    inverse_kinematics_ops bound = baseline().ik;
    bound.inverse_kinematics     = solving;

    return bound;
}

inline robot_ops robot_bound_to(decltype(robot_ops::ik_solve_pose) at_the_tool, decltype(robot_ops::ik_solve_flange_pose) at_the_flange)
{
    robot_ops bound            = baseline().robot;
    bound.ik_solve_pose        = at_the_tool;
    bound.ik_solve_flange_pose = at_the_flange;

    return bound;
}

// One row with the pair a run compares over it. The two pointers are the shapes the row's own
// comparator casts them back to: an inverse kinematics aggregate for the first, a robot for the rest.
struct solve_row
{
    const evaluation::slot_evaluation &row;
    const void *first;
    const void *second;
};

inline std::array<solve_row, 3> the_solve_rows(const inverse_kinematics_ops &chain_first, const inverse_kinematics_ops &chain_second, const robot_ops &robot_first,
                                               const robot_ops &robot_second)
{
    return {solve_row{named(inverse_kinematics_evaluations().slots, "ik.inverse_kinematics"), &chain_first, &chain_second},
            solve_row{named(robot_evaluations().slots, "robot.ik_solve_pose"), &robot_first, &robot_second},
            solve_row{named(robot_evaluations().slots, "robot.ik_solve_flange_pose"), &robot_first, &robot_second}};
}

}

#endif
