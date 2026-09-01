#ifndef HPP_GUARD_PRAXIS_CONFIG_STORE_H
#define HPP_GUARD_PRAXIS_CONFIG_STORE_H

#include "praxis/config/error.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include "praxis/compat/expected.h"

#include <cstdint>
#include <optional>
#include <filesystem>

namespace praxis::config {

// What was asked for, and where it landed.
struct location
{
    std::filesystem::path given;
    std::filesystem::path resolved;
};

// An absolute path resolves to its weakly canonical form; anything else resolves against `base`
// rather than the process's working directory. A leading `~` alone names the user's home directory
// and the segments after it hang below it; a tilde followed by a name is an ordinary directory
// component; and where no home directory can be read the path resolves as written, with the tilde
// left in place.
location resolve(const std::filesystem::path &given, const std::filesystem::path &base);

// What a document is expected to carry: every value the declaration names, against whatever the
// document happens to carry.
enum class expectation : std::uint8_t
{
    complete,
    partial,
};

expected<document, error> load(const declaration &shape, const location &at);

// A document that reads whatever became of the file, and the refusal that kept the file from being
// used, when one did. `values` answers every declared key: the file's value where the file carried
// one, the declared fallback everywhere else.
struct outcome
{
    document values;
    std::optional<error> failure;
};

// What the document is expected to carry decides where a message about a value it does not carry
// lands.
outcome load_or_defaults(const declaration &shape, const location &at, expectation carries = expectation::complete);

// A starter document written from the declaration alone, each declared field carrying the fallback
// the declaration named. It refuses rather than replacing anything already at `target`, and lands
// through a rename, so an interrupted write leaves no half-written document behind.
expected<void, error> write_template(const declaration &shape, const std::filesystem::path &target);

}

#endif
