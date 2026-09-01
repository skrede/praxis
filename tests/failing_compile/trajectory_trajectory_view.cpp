#include "praxis/trajectory.h"

namespace praxis::trajectory::probe {

capability_view trajectory_view()
{
    return view_of(trajectory_ops{});
}

}
