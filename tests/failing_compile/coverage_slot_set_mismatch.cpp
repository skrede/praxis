#include "praxis/manipulator.h"

#include <string>

namespace praxis::manipulator::probe {

std::string mismatched_names()
{
    const differential_kinematics_ops described{};
    const robot_slot_set defaulted;

    return joined_slot_names(described, defaulted);
}

}
