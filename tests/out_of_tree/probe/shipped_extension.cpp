#include "shipped_extension.h"

#include "praxis/manipulator/capabilities.h"

#include "praxis/extension/coverage.h"

#include <array>
#include <cstddef>
#include <string_view>

namespace outside {

// An extension shipped by praxis, reached from outside the repository through the generated targets
// file: its slot table is read over a composition left at its defaults and over the reference
// implementation the extension carries, which resolves out of that module's archive.
std::string_view the_shipped_manipulator_reports_its_slots()
{
    const praxis::manipulator::capabilities defaulted{};
    const praxis::manipulator::capabilities referenced = praxis::manipulator::baseline();
    const std::array<praxis::capability_view, 7> left  = praxis::manipulator::capability_views(defaulted);
    const std::array<praxis::capability_view, 7> bound = praxis::manipulator::capability_views(referenced);

    if(praxis::count_defaults(left) == 0u)
        return "the shipped manipulator left at its defaults reported no defaulted slot";
    if(praxis::count_defaults(bound) != 0u)
        return "the shipped manipulator's reference implementation left a slot at its default";

    return {};
}

}
