#include "praxis/manipulator.h"

namespace praxis::manipulator::probe {

capability_view forward_kinematics_view()
{
    return view_of(forward_kinematics_ops{});
}

}
