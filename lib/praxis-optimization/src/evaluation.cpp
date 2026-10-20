#include "evaluation_tables.h"

#include "praxis/optimization/evaluation.h"

#include <array>

namespace praxis::optimization {

std::array<evaluation::evaluation_view, 1> evaluation_views(const capabilities &first, const capabilities &second)
{
    return {evaluation::evaluation_view::of(first.solver, second.solver, solver_evaluations())};
}

}
