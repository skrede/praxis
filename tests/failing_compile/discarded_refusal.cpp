#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

namespace praxis::probe {

namespace {

expected<int, refusal> valued()
{
    return 4;
}

}

void drop_valued_refusal()
{
    valued();
}

}
