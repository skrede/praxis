#include "praxis/manipulator.h"

namespace praxis::manipulator::probe {

capability_view inverse_kinematics_view()
{
    return view_of(inverse_kinematics_ops{});
}

}
