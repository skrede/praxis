#include "locator.h"
#include "source_text.h"

#include <pugixml.hpp>

#include <span>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <string_view>

namespace praxis::config {
namespace {

std::vector<step> steps_of(std::string_view key)
{
    std::vector<step> walked;
    for(std::size_t from = 0;;)
    {
        const std::size_t cut = key.find('/', from);
        if(cut == std::string_view::npos)
        {
            walked.push_back(parsed(key.substr(from)));
            return walked;
        }
        walked.push_back(parsed(key.substr(from, cut - from)));
        from = cut + 1;
    }
}

// A leaf is either an attribute of the element the key's earlier segments reach, or a child element
// of that name carrying the value as its text; a child element carrying anything else is a place
// this module does not write, and an attribute that element does not carry is one it creates.
std::optional<placement> under(std::string_view source, pugi::xml_node holding, const step &leaf)
{
    if(const pugi::xml_attribute carried = holding.attribute(leaf.name.c_str()); carried)
        return attribute_bytes(source, offset_of(holding), leaf.name, carried.value());

    const pugi::xml_node child = reached(holding, leaf);
    if(!child)
        return absent_attribute_bytes(source, offset_of(holding), leaf.name);
    if(const pugi::xml_node held = child.first_child(); held)
        return held.type() == pugi::node_pcdata ? std::optional<placement>(text_bytes(source, offset_of(held), held.value())) : std::nullopt;
    return vacant_bytes(source, offset_of(child), leaf.name);
}

std::optional<placement> placed(std::string_view source, pugi::xml_node root, std::string_view key)
{
    const std::vector<step> walked = steps_of(key);
    pugi::xml_node holding         = root;
    for(std::size_t taken = 0; taken + 1 < walked.size() && holding; ++taken)
        holding = reached(holding, walked[taken]);

    return holding ? under(source, holding, walked.back()) : std::nullopt;
}

}

step parsed(std::string_view part)
{
    const std::size_t bracket = part.find('[');
    if(bracket == std::string_view::npos)
        return step{std::string(part), 0};

    std::size_t ordinal = 0;
    for(const char digit : part.substr(bracket + 1))
    {
        if(digit < '0' || digit > '9')
            break;
        ordinal = ordinal * 10 + static_cast<std::size_t>(digit - '0');
    }
    return step{std::string(part.substr(0, bracket)), ordinal};
}

pugi::xml_node reached(pugi::xml_node from, const step &one)
{
    pugi::xml_node found = from.child(one.name.c_str());
    for(std::size_t skipped = 0; skipped < one.ordinal && found; ++skipped)
        found = found.next_sibling(one.name.c_str());
    return found;
}

std::size_t offset_of(pugi::xml_node held)
{
    return static_cast<std::size_t>(held.offset_debug());
}

std::vector<std::optional<placement>> locate(std::string_view source, std::span<const std::string> keys)
{
    pugi::xml_document held;
    const pugi::xml_parse_result read = held.load_buffer(source.data(), source.size(), everything_the_author_wrote);

    std::vector<std::optional<placement>> found;
    found.reserve(keys.size());
    for(const std::string &key : keys)
        found.push_back(read && !key.empty() ? placed(source, held.document_element(), key) : std::nullopt);
    return found;
}

}
