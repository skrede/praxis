#include "praxis/rigid_motion.h"

namespace praxis::rigid_motion::probe {

capability_view screw_view()
{
    return view_of(screw_ops{});
}

}
