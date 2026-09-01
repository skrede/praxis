#include "praxis/manipulator/control_mode.h"

#include <array>

namespace praxis::manipulator {

const std::array<const char *, 2> &control_mode_labels()
{
    static const std::array<const char *, 2> labels{"Preview", "Simulation"};

    return labels;
}

}
