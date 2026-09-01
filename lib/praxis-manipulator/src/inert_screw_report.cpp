#include "inert_screw_report.h"

#include "praxis/extension/coverage.h"

#include <spdlog/spdlog.h>

#include <cstddef>
#include <utility>
#include <string_view>

namespace praxis::manipulator {

std::string_view screw_slot_name(const rigid_motion::screw_ops &described, rigid_motion::screw_slot which)
{
    return slot_name(rigid_motion::view_of(described), static_cast<std::size_t>(which));
}

bool inert_and_reported(const rigid_motion::screw_ops &described, const rigid_motion::screw_slot_set &inert, rigid_motion::screw_slot which, std::string_view so_that, bool &reported)
{
    if(!inert.contains(which))
    {
        reported = false;

        return false;
    }

    if(!std::exchange(reported, true))
        spdlog::error("praxis: '{}' holds its default, so {}", screw_slot_name(described, which), so_that);

    return true;
}

}
