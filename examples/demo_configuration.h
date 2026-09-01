#ifndef HPP_GUARD_PRAXIS_EXAMPLES_DEMO_CONFIGURATION_H
#define HPP_GUARD_PRAXIS_EXAMPLES_DEMO_CONFIGURATION_H

#include "demo_documents.h"

#include "praxis/scene/log_buffer.h"
#include "praxis/scene/visualizer.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include "praxis/compat/expected.h"

#include <string>
#include <vector>

namespace praxis::demo {

// What the application keeps for itself between runs. Nothing depends on this document existing,
// and it is written without a person being asked for it.
config::declaration preferences_keyspace();

// Engaged only where the document itself carried a value, so a key it does not carry leaves the
// renderer's own choice in place. A width or a height that is not positive is no window size.
scene::visualizer::geometry preferred_geometry(const config::document &values);

scene::severity preferred_level(const config::document &values);

// An absent member of `left` contributes no edit, so a run that could not reach the window writes
// nothing about geometry rather than writing zeros.
std::vector<config::edit> preferences_edits(const scene::visualizer::geometry &left, scene::severity level);

// Where the application's own document carries the presets it offers, and the leaf under each of
// them naming the document that preset reads.
inline constexpr const char *preset_instances = "presets/preset";
inline constexpr const char *document_leaf    = "document";

// Which presets the demonstration offers and the document each of them reads.
config::declaration demonstration_keyspace();

// The identities the application's own document addresses its preset documents by. A key is opaque
// -- it names an entry in that list, not the name a person reads, which every preset document
// states for itself.
std::vector<std::string> preset_keys(const config::document &values);

// Where the document one preset reads is read from: the leaf the application's own document carries
// under that key, resolved through the resolver -- this application's own copy where it has one, the
// shipped document otherwise. The library is handed a location and searches for nothing.
expected<config::location, config::error> preset_document(const config::document &values, const std::string &key, const documents &mine);

// Every document the application's own document names, in the order it names them, so a caller that
// registers from the list opens the first without naming a preset of its own. A key naming no
// document contributes nothing.
std::vector<config::location> preset_locations(const config::document &values, const documents &mine);

}

#endif
