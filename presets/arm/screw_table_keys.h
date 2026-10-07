#ifndef HPP_GUARD_PRAXIS_PRESETS_ARM_SCREW_TABLE_KEYS_H
#define HPP_GUARD_PRAXIS_PRESETS_ARM_SCREW_TABLE_KEYS_H

#include "praxis/config/error.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include "praxis/compat/expected.h"

#include <Eigen/Core>

#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <string_view>

namespace praxis::presets::keys {

// Every key a screw table addresses is one of these names joined onto the caller's path, so a
// declared key and the edit a window writes it with cannot spell themselves differently.
struct screw_table_names
{
    static constexpr std::string_view home        = "home";
    static constexpr std::string_view joint       = "joint";
    static constexpr std::string_view index       = "index";
    static constexpr std::string_view linear      = "linear";
    static constexpr std::string_view angular     = "angular";
    static constexpr std::string_view position    = "position";
    static constexpr std::string_view orientation = "orientation";
};

std::string under(std::string_view at, std::string_view leaf);

// A row is addressed by the place its joint takes in the chain, counted from one, in the one
// spelling that place reads back as -- which is the text every lookup of the row compares against.
std::optional<std::size_t> ordinal_of(const std::string &identity);

// How far a chain of `joints` is read out to: the furthest joint any of `present` names, and no
// further out than the surplus a document may name past the chain's end. An identity naming no
// joint's place, and one naming a joint further out than that, is refused by name; an empty one is
// an instance no index addresses and is passed over.
expected<std::size_t, config::error> reach_of(const std::vector<std::string> &present, std::size_t joints);

void declare_triple(config::declaration &shape, const std::string &at);

// The instance of `collection` whose identity is `named`, addressed by the ordinal the document
// carries it at, or nothing where the document carries no such instance.
std::optional<std::string> instance_at(const config::document &values, const std::string &collection, const std::string &named);

// Component by component against `fallback`, so a document carrying part of a triple leaves the
// rest of it where the caller opened it.
Eigen::Vector3d read_triple(const config::document &values, const std::string &at, const Eigen::Vector3d &fallback);

// The shortest text that reads back as the same single-precision value, which is what a pose edited
// through the widgets is carried at.
void write_shortest(std::vector<config::edit> &into, const std::string &at, const Eigen::Vector3f &value);

// Full double precision, which is what a screw has to be carried at for a table written and read
// back to name the same six-vector.
void write_exact(std::vector<config::edit> &into, const std::string &at, const Eigen::Vector3d &value);

}

#endif
