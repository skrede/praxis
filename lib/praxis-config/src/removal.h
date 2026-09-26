#ifndef HPP_GUARD_PRAXIS_CONFIG_REMOVAL_H
#define HPP_GUARD_PRAXIS_CONFIG_REMOVAL_H

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/declaration.h"

#include "praxis/compat/expected.h"

#include <span>
#include <string>
#include <vector>
#include <cstddef>

namespace praxis::config {

// What a save has left to write once the instances it takes out are gone: the document without
// them, the edits that bind a value, every removal it was asked for, and how many instances went.
struct remainder
{
    std::string source;
    std::vector<edit> bound;
    std::vector<edit> gone;
    std::size_t taken_out = 0u;
};

// `source` without every instance carrying the identity a removal among `changes` names, and every
// bound edit addressing the place it still occupies now that the instances standing before it are
// gone. An instance goes whole, with the blanks in front of it where nothing else shares its line,
// and every other byte is left exactly as it was, so what is shortened this way can still be spliced
// by offset afterwards. A removal naming an identity the document does not carry takes nothing out.
// One naming a collection `shape` does not declare, one naming a collection standing under a
// collection another removal takes an instance out of, and a bound edit addressing an instance a
// removal takes are reported by name, and nothing at all is taken.
expected<remainder, error> taken_out_of(const declaration &shape, const location &at, std::string source, std::span<const edit> changes);

}

#endif
