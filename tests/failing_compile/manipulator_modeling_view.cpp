#include "praxis/manipulator.h"

namespace praxis::manipulator::probe {

capability_view modeling_view()
{
    return view_of(modeling_ops{});
}

}
