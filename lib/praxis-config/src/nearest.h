#ifndef HPP_GUARD_PRAXIS_CONFIG_NEAREST_H
#define HPP_GUARD_PRAXIS_CONFIG_NEAREST_H

#include <span>
#include <string>
#include <string_view>

namespace praxis::config {

// The candidate in `among` at the least edit distance from `to`, a substitution within `a-z` or within
// `0-9` costing half of any other edit; ties go to the lexicographically first, and nothing is nearest
// where `among` is empty.
std::string nearest(std::string_view to, std::span<const std::string> among);

}

#endif
