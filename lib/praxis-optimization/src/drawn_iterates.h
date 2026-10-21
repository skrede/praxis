#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_DRAWN_ITERATES_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_DRAWN_ITERATES_H

#include "surface_mesh.h"

#include "praxis/optimization/scenario_snapshot.h"

#include <Eigen/Core>

#include <vector>
#include <cstddef>

namespace praxis::optimization {

struct drawn_iterate
{
    std::size_t iterate;
    Eigen::Vector3d at;
};

// The iterates within the ranges where f was answered, placed at (x, y, -e), in the order recorded
// with the selected step's moved last. One whose center stands strictly within `apart` (positive) of
// one kept before it is left out, unless it is the run's last or the selected step's.
std::vector<drawn_iterate> drawn_iterates(const scenario_snapshot &shown, const field_placement &placed, double apart);

}

#endif
