#include "locator.h"
#include "removal.h"
#include "key_path.h"
#include "insertion.h"
#include "source_text.h"
#include "readdressing.h"

#include <pugixml.hpp>

#include <span>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <algorithm>
#include <string_view>

namespace praxis::config {
namespace {

// The identity an instance carries, read the way any leaf is: an attribute of the instance, or a
// child element of that name carrying it as its text.
std::string identity_carried(pugi::xml_node instance, const std::string &leaf)
{
    const pugi::xml_attribute carried = instance.attribute(leaf.c_str());
    return carried ? carried.value() : instance.child_value(leaf.c_str());
}

// Where the element whose name begins at `from` ends: past the `>` of its end tag, or past the `/>`
// the self-closing form ends with.
std::size_t element_ends(std::string_view source, std::size_t from)
{
    const std::size_t content = content_ends(source, from);
    if(content >= source.size())
        return source.size();
    if(source[content] == '/')
        return std::min(content + 2, source.size());

    const std::size_t closes = source.find('>', content);
    return closes == std::string_view::npos ? source.size() : closes + 1;
}

// Where the line break standing in front of `opens` with nothing but blanks between begins, at its
// carriage return where one stands before the line feed; nothing where anything else stands there.
std::size_t break_in_front(std::string_view source, std::size_t opens)
{
    const std::size_t last = opens == 0 ? std::string_view::npos : source.find_last_not_of(" \t", opens - 1);
    if(last == std::string_view::npos || source[last] != '\n')
        return std::string_view::npos;
    return last > 0 && source[last - 1] == '\r' ? last - 1 : last;
}

// Where the line break or the end of `source` following `ends` with nothing but blanks between
// begins; nothing where anything else stands there.
std::size_t break_after(std::string_view source, std::size_t ends)
{
    const std::size_t next = source.find_first_not_of(" \t", ends);
    if(next == std::string_view::npos)
        return source.size();
    return source[next] == '\r' || source[next] == '\n' ? next : std::string_view::npos;
}

// The bytes the element from `opens` to `ends` takes out: from the line break in front of it to the
// one ending its line where only blanks share that line with it, and only its own bytes otherwise.
std::pair<std::size_t, std::size_t> taken_bytes(std::string_view source, std::size_t opens, std::size_t ends)
{
    const std::size_t front = break_in_front(source, opens);
    const std::size_t after = break_after(source, ends);
    if(front == std::string_view::npos || after == std::string_view::npos)
        return {opens, ends};
    return {front, after};
}

// Every instance of the collection `gone` names that carries the identity it names.
std::vector<taken> instances_carrying(pugi::xml_node root, std::string_view source, const std::string &identity, const edit &gone)
{
    const std::vector<std::string_view> parts = segments_of(gone.key);
    const pugi::xml_node holding              = holder_of(root, parts);
    const step one                            = parsed(parts.back());

    std::vector<taken> found;
    std::size_t ordinal = 0u;
    for(pugi::xml_node child = holding.child(one.name.c_str()); child; child = child.next_sibling(one.name.c_str()), ++ordinal)
    {
        if(identity_carried(child, identity) != gone.value)
            continue;

        const std::size_t named     = offset_of(child);
        const auto [begins, finish] = taken_bytes(source, source.rfind('<', named), element_ends(source, named));
        found.push_back(taken{gone.key, ordinal, begins, finish - begins});
    }
    return found;
}

bool already_taken(std::span<const taken> gone, const taken &one)
{
    return std::any_of(gone.begin(), gone.end(), [&one](const taken &before) { return before.begin == one.begin; });
}

std::string without_instances(std::string source, std::vector<taken> gone)
{
    std::sort(gone.begin(), gone.end(), [](const taken &left, const taken &right) { return left.begin > right.begin; });
    for(const taken &one : gone)
        source.erase(one.begin, one.length);

    return source;
}

error nothing_taken(const location &at, const std::string &complaint)
{
    return error{error_code::unlocatable_key, "the configuration at " + at.resolved.string() + " " + complaint + ", so nothing at all was written into it or taken out of it"};
}

// Whether the last segment of `key` names one instance rather than a collection.
bool names_an_instance(const std::string &key)
{
    const std::size_t slash = key.rfind('/');
    return key.find('[', slash == std::string::npos ? 0u : slash + 1u) != std::string::npos;
}

// The removals among `changes` naming something `shape` declares no collection at, or one instance
// rather than a collection, named together the way a save names every key it can write nothing at.
std::string undeclared_collections(const declaration &shape, std::span<const edit> changes)
{
    std::string named;
    for(const edit &one : changes)
        if(one.kind == edit_kind::taken_out && (names_an_instance(one.key) || !keyed_by(shape, declared_path(one.key))))
            named += named.empty() ? one.key : ", " + one.key;

    return named;
}

// The instances the removals among `changes` name that `source` carries, in the order they were
// named. An instance named more than once, however its removal was spelled, is one instance.
std::vector<taken> instances_taken(const declaration &shape, std::string_view source, std::span<const edit> changes)
{
    pugi::xml_document held;
    if(!held.load_buffer(source.data(), source.size(), everything_the_author_wrote))
        return {};

    std::vector<taken> found;
    for(const edit &one : changes)
    {
        const std::optional<std::string> identity = one.kind == edit_kind::bound ? std::nullopt : keyed_by(shape, declared_path(one.key));
        if(!identity)
            continue;

        for(const taken &instance : instances_carrying(held.document_element(), source, *identity, one))
            if(!already_taken(found, instance))
                found.push_back(instance);
    }

    return found;
}

}

std::string still_carried(const declaration &shape, std::string_view source, std::span<const edit> changes)
{
    for(const edit &one : changes)
        if(one.kind == edit_kind::taken_out && !instances_taken(shape, source, std::span<const edit>(&one, 1u)).empty())
            return "'" + one.value + "' was taken out of '" + one.key + "' and is still carried there";

    return std::string();
}

expected<remainder, error> taken_out_of(const declaration &shape, const location &at, std::string source, std::span<const edit> changes)
{
    if(const std::string undeclared = undeclared_collections(shape, changes); !undeclared.empty())
        return unexpected(nothing_taken(at, "declares no collection at " + undeclared));

    const std::vector<taken> gone = instances_taken(shape, source, changes);
    for(const std::string &refused : {moved_collection(changes, gone), written_into_taken(changes, gone)})
        if(!refused.empty())
            return unexpected(nothing_taken(at, refused));

    remainder left{without_instances(std::move(source), gone), {}, {}, gone.size()};
    for(const edit &one : changes)
    {
        if(one.kind == edit_kind::taken_out)
            left.gone.push_back(one);
        else
            left.bound.push_back(edit{renumbered(one.key, gone), one.value});
    }

    return left;
}

}
