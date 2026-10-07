#ifndef HPP_GUARD_PRAXIS_CONFIG_FOLD_H
#define HPP_GUARD_PRAXIS_CONFIG_FOLD_H

#include "engine.h"

#include "praxis/config/error.h"
#include "praxis/config/declaration.h"

#include "praxis/compat/expected.h"

#include <string>
#include <vector>
#include <filesystem>
#include <string_view>

namespace praxis::config {

// One document read into keys, each instance keyed by its element position among its same-named
// siblings: `entries` holds every value at a declared path; `instances` the key of every collection
// instance element in document order, whether or not it carries a value; `undeclared` the key of
// every path nothing declares, however it is written; `malformed` every fault in the structure of the
// document's declared paths.
struct folding
{
    detail::entry_map entries;
    std::vector<std::string> instances;
    std::vector<std::string> undeclared;
    std::vector<std::string> malformed;
};

// `source` read as the document of `shape`'s space; a document that does not parse, does not have
// exactly one root element, or has a root other than the space is refused here.
expected<folding, error> folded(std::string_view source, const declaration &shape, const std::filesystem::path &from);

}

#endif
