#ifndef HPP_GUARD_PRAXIS_PRESETS_ARM_ARM_KEYS_H
#define HPP_GUARD_PRAXIS_PRESETS_ARM_ARM_KEYS_H

#include "screw_table_keys.h"

#include "praxis/config/document.h"

#include <span>
#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <string_view>

namespace praxis::presets::keys {

// Both tables are in their enumeration's own order, which is what reading one back as a position and
// casting relies on.
inline constexpr std::array<const char *, 3> evaluation_policies{"fail", "warn", "skip"};
inline constexpr std::array<const char *, 3> missing_asset_policies{"fail", "warn", "skip"};

inline constexpr const char *preset_name_key      = "preset/name";
inline constexpr const char *preset_scenario_key  = "preset/scenario";
inline constexpr const char *description_path_key = "description/path";
inline constexpr const char *screw_table_key      = "screw_table/document";

std::vector<std::string> spelled(std::span<const char *const> labels);

std::string text_at(const config::document &values, const std::string &key);

double real_at(const config::document &values, const std::string &key);

std::string keyed(const config::document &values, const std::string &collection, const std::string &identity, std::string_view leaf);

// The position `key`'s value stands at among `labels`, and nothing where it stands at none.
std::optional<std::size_t> spelled_index(const config::document &values, const std::string &key, std::span<const char *const> labels);

}

#endif
