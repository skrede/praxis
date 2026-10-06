#ifndef HPP_GUARD_PRAXIS_CONFIG_READ_BACK_H
#define HPP_GUARD_PRAXIS_CONFIG_READ_BACK_H

#include "praxis/config/error.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include "praxis/compat/expected.h"

#include <span>
#include <string>
#include <optional>
#include <filesystem>
#include <string_view>

namespace praxis::config {

// The kind `key` was declared with, found by dropping the ordinals a written key carries inside a
// collection, which the declaration never had.
field_kind declared_kind(const declaration &shape, const std::string &key);

// What the document reads at `key`, put back into text through the conversions the value was
// written by, so it can be compared with a written text and named in a message.
std::optional<std::string> reading(const document &reloaded, field_kind kind, const std::string &key);

// Whether two texts are one value as `kind` reads them: "-0" and "0" are one real while two reals
// however close stay two, a text and a choice compare as written, and a text that does not read as
// its kind is one value with nothing, itself included.
bool one_value(field_kind kind, std::string_view one, std::string_view other);

// The document at `candidate` loaded through this module's own load, with every one of `keys`
// required to read as the matching entry of `values` does, and its bytes required to carry no
// instance a removal among `gone` names under the parent that removal's key addresses. Each key
// agrees where what reads back is one value of its declared kind with what was written, and a
// disagreement is reported by naming the key together with what was written and what came back.
expected<void, error> reads_as_written(const declaration &shape, const std::filesystem::path &candidate, std::span<const std::string> keys, std::span<const std::string> values,
                                       std::span<const edit> gone);

}

#endif
