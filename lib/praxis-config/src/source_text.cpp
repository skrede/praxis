#include "source_text.h"

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

bool blank(char letter)
{
    return letter == ' ' || letter == '\t' || letter == '\n' || letter == '\r';
}

std::size_t past_blanks(std::string_view source, std::size_t from)
{
    while(from < source.size() && blank(source[from]))
        ++from;
    return from;
}

std::size_t past_name(std::string_view source, std::size_t from)
{
    while(from < source.size() && !blank(source[from]) && source[from] != '/' && source[from] != '>' && source[from] != '=')
        ++from;
    return from;
}

std::string_view instead_of(char letter, carrier form)
{
    if(letter == '&')
        return "&amp;";
    if(letter == '<')
        return "&lt;";
    if(letter == '>' && form == carrier::text)
        return "&gt;";
    if(letter == '"' && form == carrier::attribute)
        return "&quot;";
    if(letter == '\'' && form == carrier::attribute)
        return "&apos;";
    return std::string_view();
}

std::string escaped(std::string_view value, carrier form)
{
    std::string safe;
    safe.reserve(value.size());
    for(const char letter : value)
    {
        const std::string_view entity = instead_of(letter, form);
        if(entity.empty())
            safe.push_back(letter);
        else
            safe += entity;
    }
    return safe;
}

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

}

std::size_t past_tag(std::string_view source, std::size_t from)
{
    std::size_t at = past_name(source, from);
    while(at < source.size() && source[at] != '>')
    {
        if(source[at] != '"' && source[at] != '\'')
        {
            ++at;
            continue;
        }
        const std::size_t closes = source.find(source[at], at + 1);
        if(closes == std::string_view::npos)
            return source.size();
        at = closes + 1;
    }
    return at;
}

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

std::size_t break_begins(std::string_view source, std::size_t feed)
{
    return feed > 0 && source[feed - 1] == '\r' ? feed - 1 : feed;
}

std::optional<placement> attribute_bytes(std::string_view source, std::size_t from, std::string_view named, std::string current)
{
    for(std::size_t at = past_blanks(source, past_name(source, from)); at < source.size(); at = past_blanks(source, at))
    {
        if(source[at] == '/' || source[at] == '>')
            break;
        const std::size_t name_ends = past_name(source, at);
        const std::size_t equals    = past_blanks(source, name_ends);
        if(equals >= source.size() || source[equals] != '=')
            break;
        const std::size_t opens = past_blanks(source, equals + 1);
        if(opens >= source.size() || (source[opens] != '"' && source[opens] != '\''))
            break;
        const std::size_t closes = source.find(source[opens], opens + 1);
        if(closes == std::string_view::npos)
            break;
        if(source.substr(at, name_ends - at) == named)
            return placement{carrier::attribute, opens + 1, closes - opens - 1, std::move(current), std::string(), std::string()};
        at = closes + 1;
    }
    return std::nullopt;
}

placement text_bytes(std::string_view source, std::size_t from, std::string current)
{
    const std::size_t ends = std::min(source.find('<', from), source.size());
    return placement{carrier::text, from, ends - from, std::move(current), std::string(), std::string()};
}

placement vacant_bytes(std::string_view source, std::size_t from, std::string_view named)
{
    const std::size_t closes = past_tag(source, from);
    if(closes > 0 && closes < source.size() && source[closes - 1] == '/')
        return placement{carrier::text, closes - 1, 1, std::string(), ">", "</" + std::string(named)};
    return placement{carrier::text, closes + 1, 0, std::string(), std::string(), std::string()};
}

placement absent_attribute_bytes(std::string_view source, std::size_t from, std::string_view named)
{
    const std::size_t closes = past_tag(source, from);
    const std::size_t at     = closes > 0 && closes < source.size() && source[closes - 1] == '/' ? closes - 1 : closes;
    return placement{carrier::attribute, at, 0, std::nullopt, " " + std::string(named) + "=\"", "\""};
}

std::string spliced(std::string source, std::span<const placement> where, std::span<const std::string> values)
{
    std::vector<std::size_t> order(where.size());
    for(std::size_t which = 0; which < order.size(); ++which)
        order[which] = which;

    // Later in the document first, so an earlier replacement never moves a place still to be edited.
    std::stable_sort(order.begin(), order.end(), [where](std::size_t left, std::size_t right) { return where[left].begin > where[right].begin; });
    for(const std::size_t which : order)
        source.replace(where[which].begin, where[which].length, where[which].opening + escaped(values[which], where[which].form) + where[which].closing);
    return source;
}

}
