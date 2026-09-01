#include "praxis/rigid_motion.h"

namespace praxis::rigid_motion::probe {

capability_view frame_view()
{
    return view_of(frame_ops{});
}

}
