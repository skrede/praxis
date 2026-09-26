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

// Where the element opening at `opens` is taken from: the blanks in front of it where nothing else
// shares its line, so the line goes with it rather than being left behind empty.
std::size_t taken_from(std::string_view source, std::size_t opens)
{
    const std::size_t line = opens == 0 ? std::string_view::npos : source.rfind('\n', opens - 1);
    return line != std::string_view::npos && source.find_first_not_of(" \t", line + 1) == opens ? line : opens;
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

        const std::size_t named = offset_of(child);
        const std::size_t from  = taken_from(source, source.rfind('<', named));
        found.push_back(taken{gone.key, ordinal, from, element_ends(source, named) - from});
    }
    return found;
}

bool already_taken(std::span<const taken> gone, const taken &one)
{
    return std::any_of(gone.begin(), gone.end(), [&one](const taken &before) { return before.stem == one.stem && before.ordinal == one.ordinal; });
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

// The removals among `changes` naming something `shape` declares no collection at, named together
// the way a save names every key it can write nothing at.
std::string undeclared_collections(const declaration &shape, std::span<const edit> changes)
{
    std::string named;
    for(const edit &one : changes)
        if(one.kind == edit_kind::taken_out && !keyed_by(shape, declared_path(one.key)))
            named += named.empty() ? one.key : ", " + one.key;

    return named;
}

// The instances the removals among `changes` name that `source` carries, in the order they were
// named. An instance named twice is one instance.
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

expected<remainder, error> taken_out_of(const declaration &shape, const location &at, std::string source, std::span<const edit> changes)
{
    if(const std::string undeclared = undeclared_collections(shape, changes); !undeclared.empty())
        return unexpected(nothing_taken(at, "declares no collection at " + undeclared));

    const std::vector<taken> gone = instances_taken(shape, source, changes);
    for(const std::string &refused : {moved_collection(gone), written_into_taken(changes, gone)})
        if(!refused.empty())
            return unexpected(nothing_taken(at, refused));

    remainder left;
    left.source    = without_instances(std::move(source), gone);
    left.taken_out = gone.size();
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
