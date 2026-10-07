#ifndef HPP_GUARD_PRAXIS_CONFIG_CHECK_H
#define HPP_GUARD_PRAXIS_CONFIG_CHECK_H

#include "fold.h"

#include "praxis/config/error.h"
#include "praxis/config/declaration.h"

#include <string>
#include <vector>
#include <utility>
#include <optional>
#include <filesystem>

namespace praxis::config {

// Every finding that keeps `walked` from being used as a document of `shape`, in one refusal: a
// malformed source where any finding is structural, rejected content otherwise, and nothing where
// there is none. The refusal also names each undeclared path with its nearest declared path.
std::optional<error> refused_content(const folding &walked, const declaration &shape, const std::filesystem::path &from);

// Each path `walked` carries a value at that `shape` does not declare, once, with the declared path
// nearest it, empty where `shape` declares none.
std::vector<std::pair<std::string, std::string>> left_out(const folding &walked, const declaration &shape);

}

#endif
