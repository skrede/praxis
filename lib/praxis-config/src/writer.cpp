#include "locator.h"
#include "removal.h"
#include "insertion.h"
#include "read_back.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"

#include <spdlog/spdlog.h>

#include <span>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <utility>
#include <optional>
#include <filesystem>
#include <string_view>
#include <system_error>

namespace praxis::config {
namespace {

// What is still to be written once the edits the document already agrees with are dropped.
struct pending_write
{
    std::string base;
    std::vector<std::string> keys;
    std::vector<std::string> values;
    std::vector<placement> places;
};

std::optional<std::string> slurped(const std::filesystem::path &from)
{
    std::ifstream in(from, std::ios::binary);
    if(!in)
        return std::nullopt;

    std::ostringstream all;
    all << in.rdbuf();
    return all.str();
}

bool spilled(const std::filesystem::path &to, const std::string &text)
{
    std::ofstream out(to, std::ios::trunc | std::ios::binary);
    out << text;
    out.close();
    return static_cast<bool>(out);
}

// The candidate is hidden and does not carry the target's path unbroken, so a message naming one
// can never be read as a message naming the other.
std::filesystem::path staging_beside(const std::filesystem::path &target)
{
    return target.parent_path() / ("." + target.filename().string() + ".staging");
}

error abandoned(const std::filesystem::path &staged, error refused)
{
    std::error_code ignored;
    std::filesystem::remove(staged, ignored);
    return refused;
}

// A save is all of its edits or none of them, so every key is named at once and the document is
// left alone rather than carrying the part of the change that could be placed.
std::optional<error> unplaced(const location &at, std::span<const std::string> keys, std::span<const std::optional<placement>> found)
{
    std::string named;
    for(std::size_t which = 0; which < found.size(); ++which)
    {
        if(found[which])
            continue;
        named += named.empty() ? keys[which] : ", " + keys[which];
    }

    if(named.empty())
        return std::nullopt;
    return error{error_code::unlocatable_key, "the configuration at " + at.resolved.string() + " has no place for " + named + ", so nothing was written"};
}

std::vector<edit> admitted(const declaration &shape, const location &at, std::span<const edit> changes, write_policy policy)
{
    if(policy == write_policy::every_edit)
        return std::vector<edit>(changes.begin(), changes.end());

    const expected<document, error> present = load(shape, at);
    std::vector<edit> kept;
    for(const edit &one : changes)
    {
        if(one.kind == edit_kind::taken_out || (present && present.value().origin_of(one.key).kind == origin_kind::source))
            kept.push_back(one);
        else
            spdlog::warn("praxis: '{}' did not come from {}, so it is left unwritten", one.key, at.resolved.string());
    }
    return kept;
}

expected<pending_write, error> pending_for(const declaration &shape, const remainder &left, const location &at)
{
    std::vector<std::string> keys;
    keys.reserve(left.bound.size());
    for(const edit &one : left.bound)
        keys.push_back(one.key);

    pending_write pending;
    const std::vector<std::string> absent = absent_elements(shape, left.source, left.bound);
    pending.base                          = absent.empty() ? left.source : with_elements(left.source, absent);

    const std::vector<std::optional<placement>> found = locate(pending.base, keys);
    if(const std::optional<error> refused = unplaced(at, keys, found); refused)
        return unexpected(*refused);

    for(std::size_t which = 0; which < left.bound.size(); ++which)
    {
        if(found[which]->current == left.bound[which].value)
            continue;
        pending.keys.push_back(left.bound[which].key);
        pending.values.push_back(left.bound[which].value);
        pending.places.push_back(*found[which]);
    }
    return pending;
}

expected<void, error> landed(const declaration &shape, const location &at, const std::string &authored, const pending_write &pending, std::span<const edit> gone)
{
    if(pending.places.empty() && pending.base == authored)
        return {};

    const std::filesystem::path staged = staging_beside(at.resolved);
    if(!spilled(staged, spliced(pending.base, pending.places, pending.values)))
        return unexpected(abandoned(staged, error{error_code::unwritable_target, "nothing could be written beside the configuration at " + at.resolved.string()}));

    if(const expected<void, error> checked = reads_as_written(shape, staged, pending.keys, pending.values, gone); !checked)
    {
        spdlog::error("praxis: the configuration at {} was left as it was, because {}", at.resolved.string(), checked.error().message);
        return unexpected(abandoned(staged, checked.error()));
    }

    std::error_code renaming;
    std::filesystem::rename(staged, at.resolved, renaming);
    if(renaming)
        return unexpected(abandoned(staged, error{error_code::unwritable_target, "the configuration at " + at.resolved.string() + " could not be replaced: " + renaming.message()}));
    return {};
}

std::optional<error> refusal_among(std::span<const edit> changes)
{
    for(const edit &one : changes)
        if(one.kind == edit_kind::refused)
            return error{error_code::rejected_content, one.value};

    return std::nullopt;
}

expected<std::string, error> authored_or_created(const declaration &shape, const location &at)
{
    if(const std::optional<std::string> authored = slurped(at.resolved); authored)
        return *authored;

    if(const expected<void, error> written = write_template(shape, at.resolved); !written)
        return unexpected(written.error());

    const std::optional<std::string> created = slurped(at.resolved);
    if(!created)
        return unexpected(error{error_code::unwritable_target, "the configuration written from the declaration at " + at.resolved.string() + " could not be read"});
    return *created;
}

}

edit::edit(std::string addressed, std::string carried, edit_kind meaning)
        : key(std::move(addressed))
        , value(std::move(carried))
        , kind(meaning)
{
}

expected<void, error> save(const declaration &shape, const location &at, std::span<const edit> changes, write_policy policy)
{
    if(const std::optional<error> refused = refusal_among(changes); refused)
        return unexpected(*refused);

    const expected<std::string, error> authored = authored_or_created(shape, at);
    if(!authored)
        return unexpected(authored.error());

    const std::vector<edit> wanted        = admitted(shape, at, as_written(shape, changes), policy);
    const expected<remainder, error> left = taken_out_of(shape, at, authored.value(), wanted);
    if(!left)
        return unexpected(left.error());

    const expected<pending_write, error> pending = pending_for(shape, left.value(), at);
    if(!pending)
        return unexpected(pending.error());

    if(const expected<void, error> put = landed(shape, at, authored.value(), pending.value(), left.value().gone); !put)
        return unexpected(put.error());

    spdlog::info("praxis: {} value(s) written into the configuration at {} and {} instance(s) taken out of it", pending.value().places.size(), at.resolved.string(),
                 left.value().taken_out);
    return {};
}

}
