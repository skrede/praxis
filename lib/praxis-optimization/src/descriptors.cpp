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

constexpr std::array minimizer_descriptors{
        slot_descriptor{"minimizer.newton", [](const void *value) -> bool { return static_cast<const minimizer_ops *>(value)->newton == &inert::newton; }},
        slot_descriptor{"minimizer.gradient_descent", [](const void *value) -> bool { return static_cast<const minimizer_ops *>(value)->gradient_descent == &inert::gradient_descent; }},
};

static_assert(minimizer_descriptors.size() == static_cast<std::size_t>(minimizer_slot::count));

constexpr capability_descriptors<solver_ops> described_solvers{"optimization", solver_descriptors};
constexpr capability_descriptors<minimizer_ops> described_minimizers{"optimization", minimizer_descriptors};

}

capability_view view_of(const solver_ops &ops)
{
    return capability_view::of(ops, described_solvers);
}

capability_view view_of(const minimizer_ops &ops)
{
    return capability_view::of(ops, described_minimizers);
}

}
