#ifndef HPP_GUARD_PRAXIS_CONFIG_KEY_PATH_H
#define HPP_GUARD_PRAXIS_CONFIG_KEY_PATH_H

#include <string>
#include <vector>
#include <cstddef>
#include <string_view>

namespace praxis::config {

// Addressing inside a collection is by ordinal, so the path a read is given carries brackets the
// declaration never had; dropping them is what turns a read key back into a declared one.
std::string declared_path(std::string_view key);

std::vector<std::string_view> segments_of(std::string_view path);

std::size_t segments_in(std::string_view path);

// The first `count` segments of `path`, or the whole of it where it has no more than that many.
std::string leading_segments(const std::string &path, std::size_t count);

bool hangs_under(std::string_view path, std::string_view ancestor);

}

#endif
