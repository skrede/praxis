#include "praxis/trajectory.h"

namespace praxis::trajectory::probe {

capability_view pose_trajectory_view()
{
    return view_of(pose_trajectory_ops{});
}

}
