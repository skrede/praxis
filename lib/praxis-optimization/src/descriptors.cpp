#include "praxis/optimization/slots.h"

#include <array>
#include <cstddef>

namespace praxis::optimization {

namespace {

// An identical-code-folding linker can merge two byte-identical function bodies to one address. That
// can only report a slot as still holding its default when the supplied implementation is
// byte-identical to the inert one, in which case the report is correct.
constexpr std::array solver_descriptors{
        slot_descriptor{"solver.solve", [](const void *value) -> bool { return static_cast<const solver_ops *>(value)->solve == &inert::solve; }},
};

static_assert(solver_descriptors.size() == static_cast<std::size_t>(solver_slot::count));

constexpr capability_descriptors<solver_ops> described_solvers{"optimization", solver_descriptors};

}

capability_view view_of(const solver_ops &ops)
{
    return capability_view::of(ops, described_solvers);
}

}
