#include "read_back.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"
#include "praxis/config/configurable.h"

#include "praxis/compat/expected.h"

#include <span>
#include <string>
#include <vector>
#include <optional>

namespace praxis::config {

outcome load_or_defaults(const binding &bound)
{
    return load_or_defaults(bound.shape, bound.at, bound.carries);
}

std::vector<edit> unsaved_edits(const document &carried, std::span<const edit> changes)
{
    std::vector<edit> outstanding;
    for(const edit &change : changes)
    {
        const std::optional<field_kind> kind = carried.kind_of(change.key);
        if(!kind)
        {
            outstanding.push_back(change);
            continue;
        }

        const std::optional<std::string> read  = reading(carried, *kind, change.key);
        const std::optional<std::string> meant = canonical(*kind, change.value);
        if(!read || !meant || *read != *meant)
            outstanding.push_back(change);
    }
    return outstanding;
}

std::vector<edit> shown_edits(std::span<const configurable *const> shown, const document &carried)
{
    std::vector<edit> gathered;
    for(const configurable *const one : shown)
    {
        if(one == nullptr)
            continue;

        const std::vector<edit> mine = one->settings_edits(carried);
        gathered.insert(gathered.end(), mine.begin(), mine.end());
    }
    return gathered;
}

bool anything_unsaved(std::span<const configurable *const> shown, const document &carried)
{
    return !shown_edits(shown, carried).empty();
}

expected<void, error> save(const binding &bound, std::span<const edit> changes)
{
    return save(bound.shape, bound.at, changes);
}

}
