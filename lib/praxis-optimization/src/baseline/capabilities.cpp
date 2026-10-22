#include "praxis/optimization/capabilities.h"
#include "praxis/optimization/baseline/solver.h"
#include "praxis/optimization/baseline/minimizer.h"

#include <array>

namespace praxis::optimization {

capabilities baseline()
{
    return capabilities{
            .solver    = solver_ops{.solve = &solve},
            .minimizer = minimizer_ops{.newton = &newton, .gradient_descent = &gradient_descent},
    };
}

std::array<capability_view, 2> capability_views(const capabilities &c)
{
    return {view_of(c.solver), view_of(c.minimizer)};
}

}
