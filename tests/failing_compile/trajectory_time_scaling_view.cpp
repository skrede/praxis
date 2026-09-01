#include "praxis/trajectory.h"

namespace praxis::trajectory::probe {

capability_view time_scaling_view()
{
    return view_of(time_scaling_ops{});
}

}
