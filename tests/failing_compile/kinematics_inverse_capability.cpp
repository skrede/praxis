#include "praxis/manipulator.h"

namespace praxis::manipulator::probe {

capability_view outliving_view()
{
    return kinematics{}.ik_capability();
}

}
