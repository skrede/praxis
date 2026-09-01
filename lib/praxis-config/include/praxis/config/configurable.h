#ifndef HPP_GUARD_PRAXIS_CONFIG_CONFIGURABLE_H
#define HPP_GUARD_PRAXIS_CONFIG_CONFIGURABLE_H

#include "praxis/config/writer.h"
#include "praxis/config/document.h"

#include <span>
#include <vector>
#include <string_view>

namespace praxis::config {

// A live object carrying settings, answering for the key path its values live under and for the
// edits its current state stands for. What a document its edits belong in is, the object says
// itself, so nothing has to look it up.
class configurable
{
public:
    configurable()                                = default;
    configurable(const configurable &)            = delete;
    configurable(configurable &&)                 = delete;
    configurable &operator=(const configurable &) = delete;
    configurable &operator=(configurable &&)      = delete;
    virtual ~configurable()                       = default;

    virtual std::string_view settings_path() const = 0;

    // The edits that still have to reach `carried` for it to read back what this object stands
    // for. An implementor addressing a collection instance by name reads it to name the ordinal
    // that instance sits at.
    virtual std::vector<edit> settings_edits(const document &carried) const = 0;
};

// Which of `changes` the document does not already read as, compared through the conversions each
// key's declared kind is written by, so a real spelled two ways is one value. A key the document
// names no declared leaf for is one of them. An implementor whose reading of an absent leaf is the
// document's own narrows its offer with this; one reading such a leaf as something else answers
// against its own reference instead.
std::vector<edit> unsaved_edits(const document &carried, std::span<const edit> changes);

}

#endif
