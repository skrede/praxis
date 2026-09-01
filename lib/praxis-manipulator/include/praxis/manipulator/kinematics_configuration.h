#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_KINEMATICS_CONFIGURATION_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_KINEMATICS_CONFIGURATION_H

#include "praxis/manipulator/ik_seed_window.h"
#include "praxis/manipulator/ik_branch_window.h"
#include "praxis/manipulator/ik_iterate_window.h"
#include "praxis/manipulator/ik_convergence_window.h"

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <vector>
#include <cstddef>
#include <string_view>

namespace praxis::manipulator {

// `at` is a key path the caller owns. Each of these declares its own leaves beneath it and declares
// nothing above it. A document carrying some of them leaves the rest where the settings struct
// opens them, and one carrying no start at all leaves the list opening at the spread
// `ik_seed_window::opening_seeds` names.

// The starts are a collection keyed by their position in the list, counted from one, and each row
// carries its joint values in radians. A row whose width is not `joints` is declined by name and
// the rows beside it are still read. The write takes the document because a document carries no
// way to drop a row: a list shorter than the one already there empties the rows it no longer
// reaches, and a row carrying no value is read as one that is not there.
void declare_ik_seeds(config::declaration &shape, std::string_view at);
ik_seed_window::settings read_ik_seeds(const config::document &values, std::string_view at, std::size_t joints);
std::vector<config::edit> write_ik_seeds(const config::document &values, const ik_seed_window::settings &state, std::string_view at);

void declare_ik_branch(config::declaration &shape, std::string_view at);
ik_branch_window::settings read_ik_branch(const config::document &values, std::string_view at);
std::vector<config::edit> write_ik_branch(const ik_branch_window::settings &state, std::string_view at);

// The start the table is about is carried counted from one, as the list of starts is, so a document
// and a reader of it name the same start. A value naming no row leaves the choice where the
// settings struct opens it.
void declare_ik_iterates(config::declaration &shape, std::string_view at);
ik_iterate_window::settings read_ik_iterates(const config::document &values, std::string_view at);
std::vector<config::edit> write_ik_iterates(const ik_iterate_window::settings &state, std::string_view at);

void declare_ik_convergence(config::declaration &shape, std::string_view at);
ik_convergence_window::settings read_ik_convergence(const config::document &values, std::string_view at);
std::vector<config::edit> write_ik_convergence(const ik_convergence_window::settings &state, std::string_view at);

}

#endif
