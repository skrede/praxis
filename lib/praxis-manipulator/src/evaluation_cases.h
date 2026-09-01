#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_EVALUATION_CASES_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_EVALUATION_CASES_H

#include "praxis/manipulator/types.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/screw_chain.h"

#include "praxis/evaluation/generation.h"

#include <meios/model.h>

#include <cstddef>
#include <optional>

namespace praxis::manipulator {

struct evaluation_case
{
    screw_chain chain;
    joint_vector joints;
};

// A solver over the drawn chain, a pose that chain stands at when driven to a drawn configuration,
// the configuration it stands there at, and a second drawn configuration to start a search from. The
// pose is reached rather than drawn, so a refusal in the run is a solve that failed and not a target
// outside the workspace.
struct solve_case
{
    kinematics solver;
    transform reached;
    joint_vector standing;
    joint_vector seed;
};

std::optional<solve_case> drawn_solve(evaluation::case_source &drawn);

// A pose a chain can reach is reached by more than one configuration, so an answer is read back
// through the shared forward map before it is held against the pose it was asked for.
std::optional<transform> reached_by(const solve_case &solved, const joint_vector &answer);

// Drawn from one source in this order and no other: the joint count, the home pose, one screw per
// joint, then one angle per joint. The chain's limits carry one entry per joint, which is what
// `kinematics::compose` requires of a chain before it composes a solver over it; they are derived
// from the drawn count rather than drawn.
evaluation_case drawn_case(evaluation::case_source &drawn);

// One angle from `angle_radians` per joint.
joint_vector drawn_joints(evaluation::case_source &drawn, std::size_t joints);

// A serial revolute machine of a freshly drawn joint count, each joint about a drawn direction at a
// drawn offset from the one before it, with the topology tables written out.
meios::model<> drawn_model(evaluation::case_source &drawn);

}

#endif
