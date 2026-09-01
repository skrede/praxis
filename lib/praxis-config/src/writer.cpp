#include "locator.h"
#include "insertion.h"
#include "read_back.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"

#include <spdlog/spdlog.h>

#include <span>
#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <charconv>
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
        if(present && present.value().origin_of(one.key).kind == origin_kind::source)
            kept.push_back(one);
        else
            spdlog::warn("praxis: '{}' did not come from {}, so it is left unwritten", one.key, at.resolved.string());
    }
    return kept;
}

expected<pending_write, error> pending_for(const declaration &shape, std::string_view authored, const location &at, std::span<const edit> wanted)
{
    std::vector<std::string> keys;
    keys.reserve(wanted.size());
    for(const edit &one : wanted)
        keys.push_back(one.key);

    pending_write pending;
    const std::vector<std::string> absent = absent_elements(shape, authored, wanted);
    pending.base                          = absent.empty() ? std::string(authored) : with_elements(std::string(authored), absent);

    const std::vector<std::optional<placement>> found = locate(pending.base, keys);
    if(const std::optional<error> refused = unplaced(at, keys, found); refused)
        return unexpected(*refused);

    for(std::size_t which = 0; which < wanted.size(); ++which)
    {
        if(found[which]->current == wanted[which].value)
            continue;
        pending.keys.push_back(wanted[which].key);
        pending.values.push_back(wanted[which].value);
        pending.places.push_back(*found[which]);
    }
    return pending;
}

expected<void, error> landed(const declaration &shape, const location &at, const std::string &candidate, const pending_write &pending)
{
    const std::filesystem::path staged = staging_beside(at.resolved);
    if(!spilled(staged, candidate))
        return unexpected(abandoned(staged, error{error_code::unwritable_target, "nothing could be written beside the configuration at " + at.resolved.string()}));

    if(const expected<void, error> checked = reads_as_written(shape, staged, pending.keys, pending.values); !checked)
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

std::string exact_text(double value)
{
    std::array<char, 40> digits{};
    const std::to_chars_result printed = std::to_chars(digits.data(), digits.data() + digits.size(), value);
    return printed.ec == std::errc() ? std::string(digits.data(), printed.ptr) : std::string();
}

expected<void, error> save(const declaration &shape, const location &at, std::span<const edit> changes, write_policy policy)
{
    const expected<std::string, error> authored = authored_or_created(shape, at);
    if(!authored)
        return unexpected(authored.error());

    const std::vector<edit> wanted               = admitted(shape, at, changes, policy);
    const expected<pending_write, error> pending = pending_for(shape, authored.value(), at, wanted);
    if(!pending)
        return unexpected(pending.error());

    if(!pending.value().places.empty() || pending.value().base != authored.value())
    {
        const std::string candidate = spliced(pending.value().base, pending.value().places, pending.value().values);
        if(const expected<void, error> put = landed(shape, at, candidate, pending.value()); !put)
            return unexpected(put.error());
    }

    spdlog::info("praxis: {} value(s) written into the configuration at {}", pending.value().places.size(), at.resolved.string());
    return {};
}

}
