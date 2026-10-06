#ifndef HPP_GUARD_PRAXIS_CONFIG_CHECK_H
#define HPP_GUARD_PRAXIS_CONFIG_CHECK_H

#include "fold.h"

#include "praxis/config/error.h"
#include "praxis/config/declaration.h"

#include <optional>
#include <filesystem>

namespace praxis::config {

// Every finding that keeps `walked` from being used as a document of `shape`, in one refusal: a
// malformed source where any finding is structural, rejected content otherwise, and nothing where
// there is none.
std::optional<error> refused_content(const folding &walked, const declaration &shape, const std::filesystem::path &from);

}

#endif
