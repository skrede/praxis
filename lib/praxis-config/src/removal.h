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
#include <string_view>

namespace praxis::config {

// What a save has left to write once the instances it takes out are gone: the document without
// them, the edits that bind a value, every removal it was asked for, and how many instances went.
struct remainder
{
    std::string source;
    std::vector<edit> bound;
    std::vector<edit> gone;
    std::size_t taken_out;
};

// `source` without every instance carrying the identity a removal among `changes` names under the
// parent its key addresses, and every bound edit addressing the place it still occupies now that the
// instances standing before it are gone. Keys are compared by the place they name, a segment with no
// bracket naming the first instance. An instance goes whole: where only blanks share its line, from
// the line break in front of it, carriage return included, to the one ending that line; otherwise
// only its own bytes. Every other byte is left exactly as it was, so what is shortened this way can
// still be spliced by offset afterwards. A removal naming an identity the document does not carry
// takes nothing out. One naming a collection `shape` does not declare or one instance rather than a
// collection, one naming a collection standing under a collection another removal takes an instance
// out of, and a bound edit addressing an instance a removal takes are reported by name, and nothing
// at all is taken.
expected<remainder, error> taken_out_of(const declaration &shape, const location &at, std::string source, std::span<const edit> changes);

// What is wrong with `source` standing for a save that made the removals among `changes`, or
// nothing: the first of them whose instance it still carries under the parent its key addresses.
std::string still_carried(const declaration &shape, std::string_view source, std::span<const edit> changes);

}

#endif
