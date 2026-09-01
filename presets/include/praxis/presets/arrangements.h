#ifndef HPP_GUARD_PRAXIS_PRESETS_ARRANGEMENTS_H
#define HPP_GUARD_PRAXIS_PRESETS_ARRANGEMENTS_H

#include "praxis/scene/preset_registry.h"

#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <span>
#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <filesystem>
#include <functional>

namespace praxis::presets {

// Where a frame arrangement keeps its placements in the document its scenario is composed from.
inline constexpr const char *arrangement_path = "arrangement";

// The key path a preset's frame arrangement writes its edits under, and the document that
// arrangement opens at what it carries of. An absent document opens at the arrangement the preset
// supplies itself, and an empty path offers no edits.
struct arrangement_source
{
    std::string at;
    std::optional<config::document> values;
};

// The values one arrangement composes from, at the root of a space of its own, so a document read
// against another one is refused rather than answered from fallbacks throughout.
config::declaration arrangement_keyspace();

// The spellings a document's scenario leaf may carry, in the order of the composers they select
// from. A document naming anything else is refused where it is read.
std::span<const char *const> arrangement_scenario_labels();

// The document is named by the caller and resolved against the directory it is expected to sit
// beside: nothing is searched for, and a document that is not there is not an error.
config::binding arrangement_binding(const std::filesystem::path &named, const std::filesystem::path &beside);

// Asked where a named document is read from, at the moment a composition wants it. A name can stand
// for more than one candidate place, and which of them answers can change while the application
// runs, so a composition asks again rather than composing through the answer its registration got. A
// route that answers nothing leaves every composition on the location its preset was registered
// with, which is what a caller whose documents have one place each wants.
using document_route = std::function<config::location(const std::filesystem::path &named)>;

// Told what a composition was built from, once that composition has been answered. A caller with
// nowhere to write an edit back to supplies nothing.
using composed_route = std::function<void(const config::binding &, const config::document &)>;

// One preset per document: each of them states the name it is shown under and says which scenario
// it is, so a document that is not there is a preset that does not exist. The names registered are
// answered in the order the span carried them, and a document whose name another one has already
// taken is left out and named.
std::vector<std::string> register_arrangements(const std::shared_ptr<scene::preset_registry> &registry, std::span<const config::location> documents, document_route located,
                                               composed_route announced);

}

#endif
