#ifndef HPP_GUARD_PRAXIS_SCENE_COVERAGE_REPORT_H
#define HPP_GUARD_PRAXIS_SCENE_COVERAGE_REPORT_H

#include "praxis/extension/descriptor.h"

#include <span>

namespace praxis::scene {

// The views point into the values they were built from, which must outlive the call. The summary is
// emitted at the informational level; the per-slot enumeration behind it is emitted at the debug
// level and is assembled only when the logger admits that level.
void report_default_slots(std::span<const capability_view> views);

}

#endif
