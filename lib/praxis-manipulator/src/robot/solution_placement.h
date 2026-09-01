#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_SOLUTION_PLACEMENT_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_ROBOT_SOLUTION_PLACEMENT_H

#include "praxis/manipulator/types.h"

#include <span>
#include <vector>
#include <cstddef>
#include <optional>

namespace praxis::manipulator {

// The unweighted L2 norm over the per-joint differences, each wrapped into (-pi, pi] before it is
// squared, so two configurations a full turn apart on one joint stand no distance apart. Two
// configurations of different width share no per-joint difference to take and are infinitely far.
double joint_distance(const joint_vector &from, const joint_vector &to);

// Which of the configurations stands nearest the one given, by the distance above, and the first of
// them where two are equally near. Absent for an empty set and for one carrying a configuration
// whose width is not the given one's.
std::optional<std::size_t> nearest_solution(std::span<const joint_vector> among, const joint_vector &from);

// Which of the distinct configurations the one found belongs to, appended as one of its own where it
// stands apart from every one of them. Two configurations nearer than the fold tolerance are one
// posture; two of different widths are never one, by the distance above.
std::size_t fold_solution(std::vector<joint_vector> &distinct, const joint_vector &found);

}

#endif
