#include "praxis/scheduler/ownership.h"

namespace praxis::probe {

namespace {

scheduler::strand_owned<int> counted{scheduler::strand{}, 0};

}

void strand_owned_escape()
{
    counted.value() += 1;
}

}
