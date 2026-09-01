#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

namespace praxis::probe {

namespace {

expected<void, refusal> valueless()
{
    return {};
}

}

void drop_valueless_refusal()
{
    valueless();
}

}
