#ifndef HPP_GUARD_PRAXIS_CONFIG_WRITER_H
#define HPP_GUARD_PRAXIS_CONFIG_WRITER_H

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/declaration.h"

#include "praxis/compat/expected.h"

#include <span>
#include <string>
#include <cstdint>

namespace praxis::config {

// One value bound for one key, in the text form the document carries it in.
struct edit
{
    std::string key;
    std::string value;
};

// Which of the edits a save is willing to write. The file is the only source a value can reach a
// caller from, so writing every edit is what a save means today; the other arm is what a save has
// to mean once some source other than the file can supply a value, and it is implemented rather
// than described so that day is a changed default rather than a changed writer.
enum class write_policy : std::uint8_t
{
    every_edit,
    file_backed_only,
};

// The shortest text that reads back as the same value, which is what a real leaf has to carry for a
// save and the load after it to agree bit for bit.
std::string exact_text(double value);

// The values in `changes` written into the document at `at`, in place: only the bytes carrying an
// edited value are replaced, so every comment, blank line, ordering and indentation the author
// wrote survives. A value the document has no place for is written as an attribute of the element
// it hangs under, and that element is created where the document does not carry it. A key whose
// path the declaration does not name, and a key addressing an instance of a collection the document
// does not carry that the same save names no identity for, or names one as an empty value, are
// reported by name and nothing at all is written; where the save does name that identity as a value
// that is not empty, the instance is created at the end of the collection and the rest of its values
// are written into it. Where there is no document at `at`, one is written from the declaration
// first, each declared field carrying the fallback the declaration named, and the values are written
// into that. What would replace the document is staged beside it, loaded back through this module's
// own load and required to read what was written, and renamed onto the document only then. One
// message reports the resolved path and how many values were written.
expected<void, error> save(const declaration &shape, const location &at, std::span<const edit> changes, write_policy policy = write_policy::every_edit);

}

#endif
