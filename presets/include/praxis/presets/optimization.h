#ifndef HPP_GUARD_PRAXIS_PRESETS_OPTIMIZATION_H
#define HPP_GUARD_PRAXIS_PRESETS_OPTIMIZATION_H

#include "praxis/presets/routes.h"

#include "praxis/optimization/registered.h"
#include "praxis/optimization/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <span>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <filesystem>
#include <functional>

namespace praxis::presets {

// The values one robot-free scenario composes from, at the root of a space of their own, so a
// document read against another one is refused rather than answered from fallbacks throughout.
config::declaration optimization_keyspace();

// curve solves a registered function of one variable for a root. The order is the label table's,
// which is what reading a spelling back as an index and casting relies on.
enum class optimization_scenario : std::uint8_t
{
    curve
};

// The spellings a document's scenario leaf may carry. A document naming anything else is refused
// where it is read.
std::span<const char *const> optimization_scenario_labels();

using optimization_composer = std::function<std::shared_ptr<scene::preset>(const scene::preset_site &, const config::document &)>;

// The capabilities and the registered functions are copied into the composer, so neither argument
// need outlive the call. The document selects one of the functions by its name. A composer checking
// derivatives has the Linearization window compare each registered derivative with a central
// difference of its function.
optimization_composer composer_for(optimization_scenario scenario, const optimization::capabilities &solving, std::vector<optimization::registered_curve> curves,
                                   bool checks_derivative);

// The document is named by the caller and resolved against the directory it is expected to sit
// beside: nothing is searched for, and a document that is not there is not an error.
config::binding optimization_binding(const std::filesystem::path &named, const std::filesystem::path &beside);

// One preset per document: each states the name it is shown under and which scenario it is, so a
// document that is not there is a preset that does not exist. The names registered are answered in
// the order the span carried them, and a document whose name another one has already taken is left
// out and named. Every preset is composed over the functions, the capabilities and the derivative
// check handed in here.
std::vector<std::string> register_optimization(const std::shared_ptr<scene::preset_registry> &registry, std::span<const config::location> documents, document_route located,
                                               composed_route announced, const optimization::capabilities &solving, std::span<const optimization::registered_curve> curves,
                                               bool checks_derivative);

}

#endif
