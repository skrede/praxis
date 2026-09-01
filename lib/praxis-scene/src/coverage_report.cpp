#include "praxis/scene/coverage_report.h"

#include "praxis/extension/coverage.h"

#include <spdlog/spdlog.h>

#include <span>

namespace praxis::scene {

// The guard is on the loop rather than on each line: the enumeration is built and each entry is
// formatted before the logger consults its level.
void report_default_slots(std::span<const capability_view> views)
{
    spdlog::info("{} composed slots hold an inert default", count_defaults(views));

    if(!spdlog::should_log(spdlog::level::debug))
        return;

    for(const defaulted_slot &entry : defaulted_slots(views))
        spdlog::debug("{}.{} holds an inert default", entry.extension, entry.slot);
}

}
