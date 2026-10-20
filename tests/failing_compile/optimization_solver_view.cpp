#include "praxis/optimization.h"

namespace praxis::optimization::probe {

capability_view solver_view()
{
    return view_of(solver_ops{});
}

}
