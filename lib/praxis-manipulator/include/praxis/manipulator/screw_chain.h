#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_CHAIN_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_SCREW_CHAIN_H

#include "praxis/manipulator/types.h"

#include "praxis/rigid_motion/types.h"

#include <vector>
#include <cstddef>

namespace praxis::manipulator {

// The screw axes are expressed in the frame of the model's root link, which is also the frame the
// rendered robot is expressed in.
struct screw_chain
{
    transform home;
    std::vector<screw_axis> space_screws;
    joint_limits limits;

    screw_chain();
    screw_chain(transform home_pose, std::vector<screw_axis> screws, joint_limits bounds);

    std::size_t joint_count() const;
};

}

#endif
