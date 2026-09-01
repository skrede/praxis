#include "praxis/scheduler/strand.h"

namespace praxis::probe {

namespace {

const scheduler::strand unbound{};

}

void sample_time_transposed()
{
    unbound.every(scheduler::every_step, scheduler::overrun::drop, [](scheduler::sample_time) {});
}

}
