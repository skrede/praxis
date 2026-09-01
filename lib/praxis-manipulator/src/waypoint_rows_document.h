#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_WAYPOINT_ROWS_DOCUMENT_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_WAYPOINT_ROWS_DOCUMENT_H

#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include <span>
#include <string>
#include <vector>
#include <string_view>

namespace praxis::manipulator::waypoint_rows {

// What every list of rows in a document spells the same way. The leaf the numbers themselves are
// held under is the caller's, because that is what one kind of row spells differently from another.
struct leaf_names
{
    static constexpr std::string_view row   = "waypoint";
    static constexpr std::string_view index = "index";
};

// One row as the document carries it: the identity the collection keys it by, and the numbers its
// one text field holds. A row carrying no text at all is a row a shorter list left behind and is not
// among these.
struct carried_row
{
    std::string identity;
    std::vector<double> values;
};

void declare(config::declaration &shape, std::string_view at, std::string_view leaf);
std::vector<carried_row> read(const config::document &values, std::string_view at, std::string_view leaf);

// The numbers of one row, spelled by the caller because how short a spelling reads back as the same
// value is a property of the type the caller holds them in, separated by single spaces.
std::string joined(std::span<const std::string> numbers);

// The document is taken because it carries no way to drop a row: a list shorter than the one already
// there empties the rows it no longer reaches.
std::vector<config::edit> write(const config::document &values, std::span<const std::string> rows, std::string_view at, std::string_view leaf);

}

#endif
