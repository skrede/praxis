#ifndef HPP_GUARD_PRAXIS_CONFIG_READ_BACK_H
#define HPP_GUARD_PRAXIS_CONFIG_READ_BACK_H

#include "praxis/config/error.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include "praxis/compat/expected.h"

#include <span>
#include <string>
#include <optional>
#include <filesystem>

namespace praxis::config {

// The kind `key` was declared with, found by dropping the ordinals a written key carries inside a
// collection, which the declaration never had.
field_kind declared_kind(const declaration &shape, const std::string &key);

// What the document reads at `key`, put back into text through the conversions the value was
// written by, so a comparison is between two texts and a message can name both.
std::optional<std::string> reading(const document &reloaded, field_kind kind, const std::string &key);

// A written text put through the same conversions, so "1.50" and "1.5" are the same real while a
// real that came back a bit short is not, and a text that is not of its kind at all is nothing.
std::optional<std::string> canonical(field_kind kind, const std::string &value);

// The document at `candidate` loaded through this module's own load, with every one of `keys`
// required to read as the matching entry of `values` does. Each key is read as the kind it was
// declared with, so a real leaf agrees only where the two are the same value bit for bit, and a
// disagreement is reported by naming the key together with what was written and what came back.
expected<void, error> reads_as_written(const declaration &shape, const std::filesystem::path &candidate, std::span<const std::string> keys, std::span<const std::string> values);

}

#endif
