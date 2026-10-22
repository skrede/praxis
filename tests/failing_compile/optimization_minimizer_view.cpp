#include "praxis/optimization.h"

namespace praxis::optimization::probe {

capability_view minimizer_view()
{
    return view_of(minimizer_ops{});
}

}
