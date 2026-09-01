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

// The chain `values` carries under `at`, one row per joint of `derived`. A row the document carries
// no instance of opens at the degenerate screw for the joint standing there, and a table naming a
// joint the chain does not have is refused with both counts rather than remapped onto it.
expected<manipulator::screw_modeling_window::settings, config::error> read_screw_table(const config::document &values, std::string_view at, const manipulator::screw_chain &derived,
                                                                                       const rigid_motion::screw_ops &turning, const rigid_motion::frame_ops &framing);

// The leaves of `state` that `values` does not already read as, with the identity of a row the
// document carries no instance of written ahead of that row's own values, so the ordinal the row is
// appended at is the one the rest of it is addressed by.
std::vector<config::edit> write_screw_table(const config::document &values, std::string_view at, const manipulator::screw_modeling_window::settings &state,
                                            const rigid_motion::frame_ops &framing);

// How a chain a window holds is spelled in the document it is judged against, so the window's own
// offer on leaving goes through the one writer that resolves a row by its identity.
manipulator::screw_modeling_window::edit_route screw_table_edits(const rigid_motion::frame_ops &framing);

// Where a window's chain goes, closed over the binding it belongs in so nothing that draws holds
// one. The document is read again where the chain arrives rather than kept from composition, so a
// row a previous save appended is one this save writes into rather than beside. A binding naming no
// document answers no route, which is what a window draws no save control for.
manipulator::screw_modeling_window::save_route screw_table_route(const std::optional<config::binding> &bound, const rigid_motion::frame_ops &framing);

// The key path a preset's supplied chain writes its edits under, the document that chain opens at
// what it carries of, and where an explicit save sends it. An absent document opens the chain at its
// degenerate state, an empty path offers nothing on leaving, and an empty route draws no save
// control.
struct screw_table_source
{
    std::string at;
    std::optional<config::document> values;
    manipulator::screw_modeling_window::save_route save;
};

}

#endif
