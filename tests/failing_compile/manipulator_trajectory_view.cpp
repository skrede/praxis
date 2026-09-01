#include "praxis/manipulator.h"

namespace praxis::manipulator::probe {

capability_view trajectory_view()
{
    return view_of(task_trajectory_ops{});
}

}
