#include "praxis/manipulator.h"

namespace praxis::manipulator::probe {

capability_view mismatched_view()
{
    const robot_ops ops{};
    const capability_descriptors<forward_kinematics_ops> described{"manipulator", {}};

    return capability_view::of(ops, described);
}

}
