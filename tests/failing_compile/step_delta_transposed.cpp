#include "praxis/scheduler/strand.h"

namespace praxis::probe {

namespace {

const scheduler::strand unbound{};

}

void step_delta_transposed()
{
    unbound.sample(scheduler::every_step, [](scheduler::step_delta) {});
}

}
