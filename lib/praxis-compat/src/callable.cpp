#include "praxis/compat/detail/callable.h"

#include <stdexcept>

namespace praxis::detail {

void called_without_target()
{
    throw std::logic_error("praxis: a callable holding no target was called");
}

}
