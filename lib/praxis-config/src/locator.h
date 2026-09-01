#ifndef HPP_GUARD_PRAXIS_CONFIG_LOCATOR_H
#define HPP_GUARD_PRAXIS_CONFIG_LOCATOR_H

#include "source_text.h"

#include <pugixml.hpp>

#include <span>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <string_view>

namespace praxis::config {

// The parser's defaults keep neither comments nor the whitespace between elements, and a document
// read without them cannot be written back without losing what the author put there.
constexpr unsigned int everything_the_author_wrote = pugi::parse_default | pugi::parse_comments | pugi::parse_declaration | pugi::parse_doctype | pugi::parse_pi | pugi::parse_ws_pcdata;

// One key segment: the element name, and which of the same-named siblings under one parent it means.
struct step
{
    std::string name;
    std::size_t ordinal;
};

step parsed(std::string_view part);

pugi::xml_node reached(pugi::xml_node from, const step &one);

std::size_t offset_of(pugi::xml_node held);

// Where each of `keys` sits in `source`, in the same order, answering nothing where the document
// carries no such place. A key is `/`-separated below the root element, a segment may carry a
// trailing `[n]` choosing that instance among the same-named siblings under one parent, and the
// last segment names either an attribute of the element the segments before it reach or a child
// element carrying the value as its text. An attribute an element the document does carry writes
// nowhere is answered as the place it would go rather than as nothing.
std::vector<std::optional<placement>> locate(std::string_view source, std::span<const std::string> keys);

}

#endif
