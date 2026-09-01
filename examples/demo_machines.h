#ifndef HPP_GUARD_PRAXIS_EXAMPLES_DEMO_MACHINES_H
#define HPP_GUARD_PRAXIS_EXAMPLES_DEMO_MACHINES_H

#include "demo_documents.h"
#include "demo_write_back.h"

#include "praxis/scene/preset_registry.h"

#include "praxis/config/document.h"

#include <string>
#include <memory>
#include <vector>
#include <filesystem>

namespace praxis::demo {

// Which of the capability slots the demonstration composes over carry a reference implementation and
// which are left on their inert default, said once for the three extensions its presets use.
void report_composed_capabilities();

// The names registered, one per document the application's own document names and in the order it
// names them, so the caller can open the first without naming a preset of its own. Each preset
// states its own name and its own scenario in the document it reads, resolved through `mine`. A
// document stating no name, or one another document has already taken, registers nothing and says so.
std::vector<std::string> register_arm_presets(const std::shared_ptr<scene::preset_registry> &registry, const config::document &values, const documents &mine,
                                              const std::filesystem::path &urdf_path, const std::shared_ptr<write_back> &through);

}

#endif
