#include "praxis/manipulator/evaluation.h"

#include <array>

namespace praxis::manipulator::probe {

std::array<evaluation::evaluation_view, 3> views_over_a_temporary()
{
    const capabilities held = baseline();

    return evaluation_views(capabilities{}, held);
}

}
