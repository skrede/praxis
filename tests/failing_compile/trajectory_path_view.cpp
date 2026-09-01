#include "praxis/trajectory.h"

namespace praxis::trajectory::probe {

capability_view path_view()
{
    return view_of(path_ops{});
}

}
