#ifndef HPP_GUARD_PRAXIS_EXAMPLES_DEMO_OFFERED_H
#define HPP_GUARD_PRAXIS_EXAMPLES_DEMO_OFFERED_H

#include "demo_documents.h"
#include "demo_write_back.h"

#include "praxis/scene/preset_registry.h"

#include "praxis/config/document.h"

#include <string>
#include <memory>
#include <vector>
#include <filesystem>

namespace praxis::demo {

// The presets registered, in the order they were registered, which is what the run opens on: the
// caller opens the first without naming a scenario of its own. The definition belongs to the
// application being linked, so which presets those are follows from the link rather than from here.
std::vector<std::string> register_offered(const std::shared_ptr<scene::preset_registry> &registry, const documents &mine, const config::document &values,
                                          const std::filesystem::path &packages, const std::shared_ptr<write_back> &writing);

}

#endif
