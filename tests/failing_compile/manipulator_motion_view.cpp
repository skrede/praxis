#include "praxis/manipulator.h"

namespace praxis::manipulator::probe {

capability_view motion_view()
{
    return view_of(motion_ops{});
}

}
