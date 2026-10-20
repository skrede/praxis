#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/baseline/solver.h"

#include <array>

namespace praxis::optimization {

capabilities baseline()
{
    return capabilities{
            .solver = solver_ops{.solve = &solve},
    };
}

std::array<capability_view, 1> capability_views(const capabilities &c)
{
    return {view_of(c.solver)};
}

}
