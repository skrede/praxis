#include "locator.h"
#include "key_path.h"
#include "insertion.h"
#include "source_text.h"

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

// One markup construct consumed from the `<` at `opens`, with `depth` following the element nesting
// across it, so a comment, a processing instruction and a character-data section carry no nesting.
std::size_t stepped_over(std::string_view source, std::size_t opens, std::size_t &depth)
{
    if(source.compare(opens, 4, "<!--") == 0)
        return std::min(source.find("-->", opens + 4), source.size());
    if(source.compare(opens, 9, "<![CDATA[") == 0)
        return std::min(source.find("]]>", opens + 9), source.size());
    if(opens + 1 >= source.size())
        return source.size();
    if(source[opens + 1] == '?' || source[opens + 1] == '!')
        return std::min(source.find('>', opens + 1), source.size());
    if(source[opens + 1] == '/')
    {
        --depth;
        return std::min(source.find('>', opens + 1), source.size());
    }

    const std::size_t closes = past_tag(source, opens + 1);
    if(!(closes > 0 && closes < source.size() && source[closes - 1] == '/'))
        ++depth;
    return closes;
}

// Where the content of the element whose name begins at `from` ends: the `<` of its end tag, or the
// byte its self-closing `/` sits on where it has no content to end.
std::size_t content_ends(std::string_view source, std::size_t from)
{
    const std::size_t closes = past_tag(source, from);
    if(closes > 0 && closes < source.size() && source[closes - 1] == '/')
        return closes - 1;

    std::size_t depth = 1;
    for(std::size_t at = closes; at < source.size();)
    {
        const std::size_t opens = source.find('<', at + 1);
        if(opens == std::string_view::npos)
            break;
        at = stepped_over(source, opens, depth);
        if(depth == 0)
            return opens;
    }
    return source.size();
}

// The blanks the line `opens` sits on begins with, and nothing where that line begins at `opens`.
std::string indent_at(std::string_view source, std::size_t opens)
{
    const std::size_t line   = opens == 0 ? std::string_view::npos : source.rfind('\n', opens - 1);
    const std::size_t begins = line == std::string_view::npos ? 0 : line + 1;
    const std::size_t ends   = std::min(source.find_first_not_of(" \t", begins), opens);
    return ends <= begins ? std::string() : std::string(source.substr(begins, ends - begins));
}

// One level of the indentation the document already uses, taken from the first line it indents.
std::string one_level(std::string_view source)
{
    for(std::size_t line = source.find('\n'); line != std::string_view::npos; line = source.find('\n', line + 1))
    {
        const std::size_t begins = source.find_first_not_of(" \t", line + 1);
        if(begins != std::string_view::npos && begins > line + 1)
            return std::string(source.substr(line + 1, begins - line - 1));
    }
    return std::string();
}

// Whether the content running from `content` to `until` is written across more than one line.
bool across_lines(std::string_view source, std::size_t content, std::size_t until)
{
    return source.substr(content, until - content).find('\n') != std::string_view::npos;
}

// Where a new last child goes in a parent whose content runs from `content` to `until`: before the
// blanks its end tag is reached over, so the whitespace already standing there stays in front of it.
std::size_t past_the_siblings(std::string_view source, std::size_t content, std::size_t until)
{
    if(!across_lines(source, content, until))
        return until;

    const std::size_t last = source.find_last_not_of(" \t\n\r", until - 1);
    return last == std::string_view::npos || last < content ? content : last + 1;
}

// The bytes an empty element `named` adds as the last child of a parent whose content runs from
// `content` to `until`: opening a line of its own at the column the siblings it joins stand at where
// that parent spans lines, and directly beside them where the parent is written on one.
std::string as_last_child(std::string_view source, std::size_t content, std::size_t until, const std::string &named)
{
    const std::string child = "<" + named + "/>";
    if(!across_lines(source, content, until))
        return child;

    const std::size_t begins = source.find_first_not_of(" \t\n\r", content);
    const std::string beside = begins < until ? indent_at(source, begins) : indent_at(source, until) + one_level(source);
    return "\n" + beside + child;
}

// A self-closing element opened around one empty child, replacing the byte its `/` sits on. The `>`
// that closed it is left to close the end tag this ends with, so nothing is written past it.
std::string opened_around(std::string_view source, pugi::xml_node holding, const std::string &named)
{
    const std::string outer = indent_at(source, offset_of(holding));
    return ">\n" + outer + one_level(source) + "<" + named + "/>\n" + outer + "</" + holding.name();
}

// The leaf the instances of the collection `shape` declares at `path` are keyed by, or nothing where
// it declares no collection there.
std::optional<std::string> keyed_by(const declaration &shape, const std::string &path)
{
    for(const node &declared : shape.nodes())
        if(declared.shape == node_kind::collection && declared.path == path)
            return declared.identity;

    return std::nullopt;
}

std::size_t children_named(pugi::xml_node holding, const char *named)
{
    std::size_t counted = 0;
    for(pugi::xml_node child = holding.child(named); child; child = child.next_sibling(named))
        ++counted;
    return counted;
}

// An identity that is empty is one nothing can address again, so it is not an identity.
bool identified(std::span<const edit> wanted, const std::string &key)
{
    return std::any_of(wanted.begin(), wanted.end(), [&key](const edit &one) { return one.key == key && !one.value.empty(); });
}

// The instances the bracketed segment ending `path` needs created for its ordinal to be reachable:
// every one from the last `holding` carries up to that ordinal, in order. An element the document
// does not carry holds none of them, so the first instance under one being created is the one at
// ordinal zero. Nothing at all where `shape` declares no collection at that path, or where `wanted`
// leaves any of them unidentified, so an instance is created only because the same edits say what
// identifies it.
std::vector<std::string> appended_instances(const declaration &shape, std::span<const edit> wanted, pugi::xml_node holding, const std::string &path)
{
    const std::optional<std::string> identity = keyed_by(shape, declared_path(path));
    if(!identity)
        return {};

    const std::string stem = path.substr(0, path.rfind('['));
    const step one         = parsed(std::string_view(path).substr(path.rfind('/') + 1));

    std::vector<std::string> made;
    for(std::size_t which = holding ? children_named(holding, one.name.c_str()) : 0u; which <= one.ordinal; ++which)
    {
        made.push_back(stem + "[" + std::to_string(which) + "]");
        if(!identified(wanted, made.back() + "/" + *identity))
            return {};
    }
    return made;
}

// The ancestors of a key's leaf that `root` does not reach, shallowest first, with an instance of a
// declared collection among them where `wanted` identifies it, or nothing at all where the chain
// spells an instance the document does not carry and nothing there names.
std::vector<std::string> missing_above(const declaration &shape, std::span<const edit> wanted, pugi::xml_node root, std::string_view key)
{
    const std::vector<std::string_view> parts = segments_of(key);
    pugi::xml_node holding                    = root;
    std::string path;
    std::vector<std::string> named;
    for(std::size_t taken = 0; taken + 1 < parts.size(); ++taken)
    {
        path += path.empty() ? std::string(parts[taken]) : "/" + std::string(parts[taken]);
        const pugi::xml_node found = holding ? reached(holding, parsed(parts[taken])) : pugi::xml_node();
        if(!found && parts[taken].find('[') != std::string_view::npos)
        {
            const std::vector<std::string> made = appended_instances(shape, wanted, holding, path);
            if(made.empty())
                return {};
            named.insert(named.end(), made.begin(), made.end());
        }
        else if(!found)
            named.push_back(path);
        holding = found;
    }
    return named;
}

// The element the last of `parts` hangs under, or nothing where the document does not carry it.
pugi::xml_node holder_of(pugi::xml_node root, const std::vector<std::string_view> &parts)
{
    pugi::xml_node holding = root;
    for(std::size_t taken = 0; taken + 1 < parts.size() && holding; ++taken)
        holding = reached(holding, parsed(parts[taken]));
    return holding;
}

std::string grown_with(std::string source, const std::string &path)
{
    pugi::xml_document held;
    if(!held.load_buffer(source.data(), source.size(), everything_the_author_wrote))
        return source;

    const std::vector<std::string_view> parts = segments_of(path);
    const pugi::xml_node holding              = holder_of(held.document_element(), parts);
    const std::size_t closes                  = holding ? past_tag(source, offset_of(holding)) : source.size();
    if(closes >= source.size())
        return source;

    const std::string named = parsed(parts.back()).name;
    if(closes > 0 && source[closes - 1] == '/')
        source.replace(closes - 1, 1, opened_around(source, holding, named));
    else
    {
        const std::size_t until = content_ends(source, offset_of(holding));
        source.insert(past_the_siblings(source, closes + 1, until), as_last_child(source, closes + 1, until, named));
    }
    return source;
}

}

std::vector<std::string> absent_elements(const declaration &shape, std::string_view source, std::span<const edit> wanted)
{
    pugi::xml_document held;
    if(!held.load_buffer(source.data(), source.size(), everything_the_author_wrote))
        return {};

    std::vector<std::string> named;
    for(const edit &one : wanted)
        for(const std::string &path : missing_above(shape, wanted, held.document_element(), one.key))
            if(std::find(named.begin(), named.end(), path) == named.end())
                named.push_back(path);
    return named;
}

std::string with_elements(std::string source, std::span<const std::string> absent)
{
    for(const std::string &path : absent)
        source = grown_with(std::move(source), path);
    return source;
}

}
