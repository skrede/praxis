#ifndef HPP_GUARD_PRAXIS_CONFIG_SOURCE_TEXT_H
#define HPP_GUARD_PRAXIS_CONFIG_SOURCE_TEXT_H

#include <span>
#include <string>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

namespace praxis::config {

// How a document carries one leaf's value, which decides what a replacement has to escape.
enum class carrier : std::uint8_t
{
    attribute,
    text,
};

// The bytes of a source document that carry one leaf's value. `current` is what those bytes say with
// entity references resolved, and nothing at all where the document carries no such place; `opening`
// and `closing` are empty wherever the value already has a place of its own -- they carry the tag
// text that gives a leaf the author self-closed one.
struct placement
{
    carrier form;
    std::size_t begin;
    std::size_t length;
    std::optional<std::string> current;
    std::string opening;
    std::string closing;
};

// The `>` closing the start tag whose element name begins at `from`, with each quoted attribute
// value stepped over whole so a `>` inside one is not mistaken for it.
std::size_t past_tag(std::string_view source, std::size_t from);

// The bytes between the quotes of the attribute `named` in the start tag whose element name begins
// at `from`, or nothing where that tag writes no such attribute.
std::optional<placement> attribute_bytes(std::string_view source, std::size_t from, std::string_view named, std::string current);

// The bytes of the text beginning at `from`, which run to the tag that ends it.
placement text_bytes(std::string_view source, std::size_t from, std::string current);

// Where a value would go in the element `named` beginning at `from`, which carries none: between
// its tags where it stands open, and in the byte the self-closing form spends on `/` where it does
// not, so writing one back opens the element rather than adding anything beside it. The element is
// there and reads as the empty string, which is the `current` this answers.
placement vacant_bytes(std::string_view source, std::size_t from, std::string_view named);

// Where the attribute `named` would go in the start tag whose element name begins at `from`, which
// writes no such attribute: immediately before the `>` that closes it, and before the `/` the
// self-closing form spends where it has one, so writing one back adds an attribute and moves nothing.
// Nothing carries the value yet, so `current` is absent rather than empty.
placement absent_attribute_bytes(std::string_view source, std::size_t from, std::string_view named);

// `source` with the value at each of `where` replaced by the matching entry of `values`, escaped
// for the form that placement carries. Every other byte is left exactly as it was.
std::string spliced(std::string source, std::span<const placement> where, std::span<const std::string> values);

}

#endif
