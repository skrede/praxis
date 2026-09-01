#include "praxis/extension/coverage.h"

#include <span>
#include <vector>
#include <cstddef>
#include <string_view>

namespace praxis {

namespace {

bool is_a_slot(const capability_view &view, std::size_t index)
{
    return index < view.slots().size();
}

}

// An identical-code-folding linker can merge two byte-identical function bodies to one address. That
// can only report a slot as still holding its default when the supplied implementation is
// byte-identical to the null one, in which case the report is correct.
std::size_t count_defaults(std::span<const capability_view> views)
{
    std::size_t held = 0;
    for(const capability_view &view : views)
    {
        for(const slot_descriptor &descriptor : view.slots())
        {
            held += descriptor.holds_default(view.value()) ? 1u : 0u;
        }
    }
    return held;
}

std::vector<defaulted_slot> defaulted_slots(std::span<const capability_view> views)
{
    std::vector<defaulted_slot> held;
    for(const capability_view &view : views)
    {
        for(const slot_descriptor &descriptor : view.slots())
        {
            if(descriptor.holds_default(view.value()))
            {
                held.push_back(defaulted_slot{view.extension(), descriptor.name});
            }
        }
    }
    return held;
}

bool holds_default(const capability_view &view, std::size_t index)
{
    return is_a_slot(view, index) && view.slots()[index].holds_default(view.value());
}

std::string_view slot_name(const capability_view &view, std::size_t index)
{
    return is_a_slot(view, index) ? view.slots()[index].name : std::string_view{};
}

}
