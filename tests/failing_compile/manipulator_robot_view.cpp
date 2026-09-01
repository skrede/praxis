#include "praxis/manipulator.h"

namespace praxis::manipulator::probe {

capability_view robot_view()
{
    return view_of(robot_ops{});
}

}
