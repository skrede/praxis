#ifndef HPP_GUARD_PRAXIS_PRESETS_SCREW_TABLE_H
#define HPP_GUARD_PRAXIS_PRESETS_SCREW_TABLE_H

#include "praxis/manipulator/screw_chain.h"
#include "praxis/manipulator/screw_modeling_window.h"

#include "praxis/config/error.h"
#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/screw.h"

#include "praxis/compat/expected.h"

#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <filesystem>
#include <string_view>

namespace praxis::presets {

// Where a supplied chain keeps its home pose and its rows in the document a scenario is composed
// from.
inline constexpr const char *screw_table_path = "screws";

// The values one supplied chain composes from, at the root of a space of its own, so a document
// read against another one is refused rather than answered from fallbacks throughout.
config::declaration screw_table_keyspace();

// The document is named by the caller and resolved against the directory it is expected to sit
// beside: nothing is searched for, and a document that is not there is not an error.
config::binding screw_table_binding(const std::filesystem::path &named, const std::filesystem::path &beside);

// The most joints past the chain's end a document may name a row for. A row addressed further out
// than that is refused, naming what could not be read, rather than stretching the reading to hold
// every joint between; a chain holding a screw further out than that is refused by the writer, naming
// that joint, rather than written into a document its reader refuses.
inline constexpr std::size_t screw_table_greatest_surplus = 64u;

// The chain `values` carries under `at`: one entry per joint of `derived`, stretched to the
// furthest joint any row names past its end so that every joint between has an entry of its own. An
// entry for a row the document carries no instance of holds nothing, because nobody supplied that
// joint. A row addressed by anything other than its joint's place in the chain, counted from one
// and spelled the way that place reads back, is refused, as is one naming a joint further out than
// the surplus reaches; either way the refusal names what could not be read.
expected<manipulator::screw_modeling_window::settings, config::error> read_screw_table(const config::document &values, std::string_view at, const manipulator::screw_chain &derived,
                                                                                       const rigid_motion::screw_ops &turning, const rigid_motion::frame_ops &framing);

// The leaves of `state` that `values` does not already read as, with the identity of a row the
// document carries no instance of written ahead of that row's own values, so the ordinal the row is
// appended at is the one the rest of it is addressed by. A row is written because somebody supplied
// that joint rather than because a leaf of it moved, so a screw every leaf of which reads as the
// fallback is a row of its own. The row of a joint whose entry holds nothing, and of every joint past
// the last entry of `state`, is taken out, so a joint nobody supplied stays a joint nobody supplied
// across a save and the open after it. A document `read_screw_table` refuses against `derived` is
// refused in its words, and a state holding a screw for a joint further out than a document may name
// is refused by naming that joint and the furthest one; either way there is nothing to write.
expected<std::vector<config::edit>, config::error> write_screw_table(const config::document &values, std::string_view at, const manipulator::screw_chain &derived,
                                                                     const manipulator::screw_modeling_window::settings &state, const rigid_motion::frame_ops &framing);

// How a chain a window holds is spelled in the document it is judged against, so the window's own
// offer on leaving goes through the one writer that resolves a row by its identity. A chain the
// writer refuses offers nothing, and the refusal is logged.
manipulator::screw_modeling_window::edit_route screw_table_edits(const manipulator::screw_chain &derived, const rigid_motion::frame_ops &framing);

// Where a window's chain goes, closed over the binding it belongs in so nothing that draws holds
// one. The document is read again where the chain arrives rather than kept from composition, so a
// row a previous save appended is one this save writes into rather than beside. A chain the writer
// refuses leaves the document as it was, and the refusal is logged. A binding naming no document
// answers no route, which is what a window draws no save control for.
manipulator::screw_modeling_window::save_route screw_table_route(const std::optional<config::binding> &bound, const manipulator::screw_chain &derived,
                                                                 const rigid_motion::frame_ops &framing);

// The key path a preset's supplied chain writes its edits under, the document that chain opens at
// what it carries of, and the binding an explicit save writes into. An absent document opens the
// chain at its degenerate state, an empty path offers nothing on leaving, and a binding naming no
// document draws no save control.
struct screw_table_source
{
    std::string at;
    std::optional<config::document> values;
    std::optional<config::binding> into;
};

}

#endif
