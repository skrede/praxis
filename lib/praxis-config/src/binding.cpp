#include "engine.h"
#include "read_back.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"
#include "praxis/config/configurable.h"

#include "praxis/compat/expected.h"

#include <spdlog/spdlog.h>

#include <span>
#include <string>
#include <vector>
#include <optional>
#include <algorithm>

namespace praxis::config {

namespace {

// An offer that is one value with the edit gathered for its key is not gathered again, so the text
// offered first stands. A different value takes that edit's place as one refusal naming both, and a
// key already refused gathers no value.
void gather(std::vector<edit> &gathered, const edit &offered, const document &carried)
{
    const field_kind kind = offered.kind == edit_kind::bound ? carried.kind_of(offered.key).value_or(field_kind::text) : field_kind::text;
    const auto alike      = [&](const edit &one) { return one.key == offered.key && one.kind == offered.kind && one_value(kind, one.value, offered.value); };
    if(std::ranges::any_of(gathered, alike))
        return;

    const auto valued = [&offered](const edit &one) { return one.key == offered.key && one.kind != edit_kind::taken_out; };
    const auto held   = std::ranges::find_if(gathered, valued);
    if(offered.kind != edit_kind::bound || held == gathered.end())
    {
        gathered.push_back(offered);
        return;
    }

    if(held->kind == edit_kind::bound)
        *held = edit{offered.key, "'" + offered.key + "' is offered as '" + held->value + "' and as '" + offered.value + "', so neither is written", edit_kind::refused};
}

bool left_unwritten(const document &carried, const edit &offered)
{
    if(offered.kind != edit_kind::bound || carried.kind_of(offered.key) != field_kind::real || !non_finite(offered.value))
        return false;

    spdlog::warn("praxis: '{}' is offered as '{}', which no document can carry, so it is left unwritten", offered.key, offered.value);
    return true;
}

}

outcome load_or_defaults(const binding &bound)
{
    return load_or_defaults(bound.shape, bound.at, bound.carries);
}

std::vector<edit> unsaved_edits(const document &carried, std::span<const edit> changes)
{
    std::vector<edit> outstanding;
    for(const edit &change : changes)
    {
        const std::optional<field_kind> kind = change.kind == edit_kind::bound ? carried.kind_of(change.key) : std::nullopt;
        if(!kind)
        {
            outstanding.push_back(change);
            continue;
        }

        const std::optional<std::string> read = reading(carried, *kind, change.key);
        if(!read || !one_value(*kind, *read, change.value))
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

        for(const edit &offered : one->settings_edits(carried))
            if(!left_unwritten(carried, offered))
                gather(gathered, offered, carried);
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
