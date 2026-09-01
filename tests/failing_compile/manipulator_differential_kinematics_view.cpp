#include "praxis/manipulator.h"

namespace praxis::manipulator::probe {

capability_view differential_kinematics_view()
{
    return view_of(differential_kinematics_ops{});
}

}
